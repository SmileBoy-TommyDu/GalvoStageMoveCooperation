# FrequencyDecomposer.Decompose / DecomposeAuto 方法详解

> 面向对象：需要理解「等时采样轨迹 → 平台低频指令 + 振镜高频指令」如何分解的开发者。
>
> 相关源码：[`FrequencyDecomposer`](../src/GalvoStage.Core/PathPlanning/FrequencyDecomposer.cs)

---

## 一、方法定位

`FrequencyDecomposer` 是激光加工链路的**第三环**：

```
DXF → DxfParser → PathSampler.Sample → [FrequencyDecomposer] → 双轴联动
```

它解决振镜平台协同的**核心矛盾**：

| 执行机构 | 行程 | 带宽（响应速度） |
|----------|------|------------------|
| XY 平台 | 大（整幅面） | 低（重、惯量大，只能走平滑低频） |
| 振镜 | 小（±视场，如 ±5mm） | 高（轻、可高频抖动） |

**分解思路**：把采样轨迹按频率拆成两部分——

- **低频分量** → 交给平台（大行程、缓慢移动）
- **高频残差** → 交给振镜（小范围、快速补偿）

两者叠加 = 原始轨迹。只要**振镜残差不超出视场**，就能用「慢平台 + 快振镜」合成出「又大又快」的加工轨迹。

---

## 二、Decompose：给定截止频率的单次分解

### 2.1 签名

```csharp
public static DecomposeResult Decompose(SampledTrajectory traj, double cutoffHz, double galvoFov)
```

| 参数 | 含义 |
|------|------|
| `traj` | 等时采样轨迹（来自 `PathSampler.Sample`） |
| `cutoffHz` | 低通截止频率（Hz）：< 此频率归平台，> 此频率归振镜 |
| `galvoFov` | 振镜视场半径（mm），仅用于结果标注是否超限 |

### 2.2 算法流程

```csharp
double fs = traj.SampleRate;
cutoffHz = Math.Clamp(cutoffHz, 0.1, fs * 0.45);       // ① 截止频率钳位

double[] stageX = FiltFilt(traj.X, cutoffHz, fs);       // ② 零相位低通 → 平台低频分量
double[] stageY = FiltFilt(traj.Y, cutoffHz, fs);

for (int i = 0; i < n; i++)                             // ③ 残差 = 原始 − 低频 → 振镜高频分量
{
    galvoX[i] = traj.X[i] - stageX[i];
    galvoY[i] = traj.Y[i] - stageY[i];
    double dev = Math.Max(Math.Abs(galvoX[i]), Math.Abs(galvoY[i]));
    if (dev > maxDev) maxDev = dev;                     //    记录振镜最大偏摆
}

(double vMax, double aMax) = KinematicStats(stageX, stageY, fs);  // ④ 平台运动学统计
```

各步骤：

- **① 截止频率钳位**：下限 0.1Hz（避免几乎不滤波），上限 `0.45×fs`（奈奎斯特安全边界，防止数值发散）。
- **② 零相位低通滤波 `FiltFilt`**：得到平台指令 `StageX/Y`。零相位（前向+反向两遍）保证滤波后**波形不发生时间偏移**——这至关重要，否则平台指令与振镜残差在时间上错位，叠加后落点就偏了。详见第四节。
- **③ 求残差**：`galvo = traj − stage`。残差就是「原始轨迹里被平台滤掉的高频细节」，正好交给振镜补。同时记录 `MaxGalvoDeviation`（X/Y 分量绝对值的最大值），用于判断是否超视场。
- **④ 运动学统计 `KinematicStats`**：对平台低频分量做数值差分，得到平台**峰值速度**与**峰值加速度**，用于评估平台是否吃得消。

### 2.3 返回值 `DecomposeResult`

| 字段 | 含义 |
|------|------|
| `StageX/Y[]` | 平台指令（低频分量） |
| `GalvoX/Y[]` | 振镜指令（高频残差，相对平台） |
| `CutoffHz` | 实际使用的截止频率 |
| `MaxGalvoDeviation` | 振镜最大偏摆（mm）——**是否 ≤ galvoFov 决定方案可行性** |
| `StageMaxVelocity` | 平台峰值速度 (mm/s) |
| `StageMaxAcceleration` | 平台峰值加速度 (mm/s²) |
| `Raw` | 原始采样轨迹引用 |

### 2.4 截止频率的权衡（核心直觉）

| 截止频率 | 平台分量 | 振镜残差 | 后果 |
|----------|----------|----------|------|
| **越高** | 跟随更多细节，速度/加速度需求大 | 残差小 | 平台可能吃不消（超速/超加速） |
| **越低** | 越平滑，速度/加速度需求小 | 残差大 | 振镜可能超视场 |

因此存在一个「最优截止频率」：**在保证振镜不超视场的前提下取尽量低的截止频率**，让平台最省力。这正是 `DecomposeAuto` 要自动搜索的目标。

---

## 三、DecomposeAuto：自动搜索最优截止频率

### 3.1 签名

```csharp
public static DecomposeResult DecomposeAuto(SampledTrajectory traj, double galvoFov,
    double margin = 0.8, double fcLow = 0.2, double fcHigh = 60)
```

| 参数 | 含义 |
|------|------|
| `galvoFov` | 振镜视场半径 (mm) |
| `margin` | 安全裕度（默认 0.8）：目标是振镜偏摆 ≤ `galvoFov × 0.8`，留 20% 余量 |
| `fcLow` / `fcHigh` | 二分搜索的截止频率下界 / 上界 |

### 3.2 目标

> 找到能使 `MaxGalvoDeviation ≤ galvoFov × margin` 的**最低**截止频率。

「最低」是因为截止频率越低平台越省力（速度/加速度越小），只要振镜还能装得下残差即可。

### 3.3 算法：单调性 + 二分查找

```csharp
double limit = galvoFov * margin;
fcHigh = Math.Min(fcHigh, traj.SampleRate * 0.45);

var high = Decompose(traj, fcHigh, galvoFov);
if (high.MaxGalvoDeviation > limit) return high;   // ① 上限都超视场 → 返回最优可行（残差最小）

var low = Decompose(traj, fcLow, galvoFov);
if (low.MaxGalvoDeviation <= limit) return low;    // ② 下限就满足 → 直接用最省力方案

DecomposeResult best = high;                       // ③ 二分：在 [fcLow, fcHigh] 找最低可行截止频率
for (int iter = 0; iter < 18 && (fcHigh - fcLow) > 0.05; iter++)
{
    double mid = 0.5 * (fcLow + fcHigh);
    var r = Decompose(traj, mid, galvoFov);
    if (r.MaxGalvoDeviation <= limit) { best = r; fcHigh = mid; }  // 可行 → 尝试更低
    else fcLow = mid;                                              // 超视场 → 必须更高
}
return best;
```

**单调性前提**：截止频率越高 → 残差越小 → `MaxGalvoDeviation` 越小。这是单调关系，因此可用二分查找。

**三种情形**：
- **① 上限仍超视场**：即使截止频率开到最高、残差已最小，振镜仍装不下。说明该轨迹对当前视场本就苛刻，返回残差最小的 `high`（最优可行近似），界面会提示"× 超出视场"。
- **② 下限即满足**：截止频率最低（平台最省力）时残差就已在视场内，直接返回 `low`。
- **③ 常规二分**：在 `[fcLow, fcHigh]` 之间二分——中点可行就压低 `fcHigh`（追求更省力），不可行就抬高 `fcLow`。

**终止条件**：最多 18 次迭代，或区间收窄到 0.05Hz 精度。18 次二分可把 60Hz 区间收敛到约 `60/2¹⁸ ≈ 0.00023Hz`，远超 0.05Hz 需求，故实际由精度条件先触发终止。

### 3.4 性能提示

`DecomposeAuto` 每次迭代都调一次 `Decompose`（含两遍 FiltFilt），即最多约 **18 次全序列滤波**。对超大轨迹开销显著——这正是 [`MainViewModel.Decompose()`](../src/GalvoStage.App/ViewModels/MainViewModel.cs) 采用「**在抽稀子集上跑 DecomposeAuto 估截止频率、再用固定频率对全量跑单次 Decompose**」两阶段策略的原因（详见 docs/03）。

---

## 四、关键子程序

### 4.1 FiltFilt：零相位二阶 Butterworth 低通

```csharp
private static double[] FiltFilt(double[] src, double fc, double fs)
```

- **零相位**：正向滤一遍 → 序列反转 → 再滤一遍 → 再反转。两遍方向相反，相位延迟相互抵消，**输出与输入时间对齐**（等效四阶幅频、零相位）。
- **边界填充**：首尾各按常值延拓 `pad = min(n−1, 2·fs/fc)` 个样本，抑制起始/结束瞬态。
- 数据不足 8 点时直接返回拷贝（无法有意义滤波）。

### 4.2 ButterLp2：二阶巴特沃斯系数（双线性变换）

```csharp
double c = 1.0 / Math.Tan(Math.PI * fc / fs);   // 预畸变
double d = 1 + √2·c + c²;
b0 = 1/d;  b1 = 2·b0;  b2 = b0;
a1 = 2(1 − c²)/d;  a2 = (1 − √2·c + c²)/d;
```

标准二阶低通差分方程系数：`c` 由截止频率经双线性变换预畸变得到，`√2` 对应巴特沃斯 Q 值（最大平坦响应）。

### 4.3 Filter：直接 II 型差分方程

```csharp
yi = b0·xi + b1·x1 + b2·x2 − a1·y1 − a2·y2;   // 二阶 IIR
```

用 `x[0]` 做稳态初始化（`x1=x2=y1=y2=x[0]`），避免起始阶跃瞬态。

### 4.4 KinematicStats：平台运动学峰值

对平台分量做一阶差分得速度、二阶差分得加速度（乘 `fs` 换算到 mm/s、mm/s²），取合成矢量模的最大值，得到 `StageMaxVelocity` / `StageMaxAcceleration`。

---

## 五、两方法对比

| 维度 | `Decompose` | `DecomposeAuto` |
|------|-------------|-----------------|
| 截止频率 | 调用方指定 | 自动二分搜索最优（最低可行） |
| 计算量 | 1 次滤波 | 最多约 18 次滤波 |
| 用途 | 已知截止频率 / 全量单次分解 | 自动整定 / 参数估计 |
| 目标 | 按给定频率拆分 | 振镜不超视场前提下平台最省力 |

---

## 六、示例

```csharp
var traj = PathSampler.Sample(polylines, feedSpeed: 80, jumpSpeedPlatform: 500, jumpSpeedGalvo: 2000, sampleRate: 1000);

// 自动整定截止频率（振镜视场 ±5mm）
var plan = FrequencyDecomposer.DecomposeAuto(traj, galvoFov: 5);
Console.WriteLine($"最优截止频率={plan.CutoffHz:F2} Hz");
Console.WriteLine($"振镜最大偏摆={plan.MaxGalvoDeviation:F3} mm（视场±5）");
Console.WriteLine($"平台峰值速度={plan.StageMaxVelocity:F1} mm/s  峰值加速度={plan.StageMaxAcceleration:F0} mm/s²");

// 或用固定截止频率做全量分解
var plan2 = FrequencyDecomposer.Decompose(traj, cutoffHz: plan.CutoffHz, galvoFov: 5);
```

---

## 附：一句话结论

> `Decompose` 用**零相位二阶 Butterworth 低通**把轨迹拆成「平台低频 + 振镜高频残差」，靠零相位保证叠加不失真；`DecomposeAuto` 利用「截止频率↑→残差↓」的单调性做**二分查找**，在振镜不超视场的前提下求出让平台最省力的最低截止频率。
