# PathSampler.Sample 方法详解（含 InterpolateSegment 深度剖析）

> 面向对象：需要理解「DXF 折线集合 → 等时采样轨迹」如何生成，尤其是逐段插补细节的开发者。
>
> 相关源码：[`PathSampler.Sample`](../src/GalvoStage.Core/PathPlanning/PathSampler.cs)、[`PathSampler.InterpolateSegment`](../src/GalvoStage.Core/PathPlanning/PathSampler.cs)

---

## 一、方法定位

`Sample` 是激光加工链路的**第二环**：

```
DXF → DxfParser → [PathSampler.Sample] → FrequencyDecomposer → 双轴联动
```

它把「一堆几何折线」转换成「**按固定时间间隔（等时）排列的落点序列**」，作为后续频率分解与联动控制的**指令基准**。输出的每个采样点代表「在第 k 个时间片，激光应到达的 XY 位置，以及此刻激光是否开启」。

### 为什么必须「等时」采样

平台与振镜是按固定控制周期（如 1kHz）接收指令的伺服系统。频率分解（filtfilt）也要求输入是**等间隔时间序列**才能做数字滤波。因此 `Sample` 的核心任务是：把「按几何长度描述的路径」重采样成「按时间片描述的轨迹」——这正是 `InterpolateSegment` 要解决的问题。

---

## 二、方法签名与输出

```csharp
public static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,           // 进给速度 mm/s（激光开，走轮廓）
    double jumpSpeedPlatform,   // 平台空移速度 mm/s（激光关）
    double jumpSpeedGalvo,      // 振镜空移速度 mm/s（激光关）
    double sampleRate,          // 采样率 Hz
    double cornerAngleDeg = 150,   // 尖角保真阈值（内角 < 此值的顶点强制吸附）；≥180 关闭
    double accelPlatform = 1000.0, // 平台加速度 mm/s²
    double accelGalvo = 5000.0,    // 振镜加速度 mm/s²
    double cornerFactor = 0.5,     // 拐角系数 0~1，尖角处速度衰减
    double decelPlatform = 0)      // 平台减速度 mm/s²（0=同 accelPlatform）
```

> **空移速度合成**：代码内部由平台/振镜两个空移速度**向量合成**为快移速度：`rapidSpeed = min(√(jumpSpeedPlatform² + jumpSpeedGalvo²), 1000)`（限幅 1000 mm/s，防采样点过疏）。旧版的单一 `rapidSpeed` 参数已拆为这两个。
> **加减速参数**（`accelPlatform/accelGalvo/decelPlatform`）已在签名中定义并有变步距插补实现（`InterpolateSegmentWithAccel`），但当前 `Sample` 主流程仍走恒速 `InterpolateSegment`；`cornerFactor` 则已生效（见 §三/§五）。

输出 `SampledTrajectory`：

| 字段 | 含义 |
|------|------|
| `X[]` / `Y[]` | 每个时间片的落点坐标 (mm) |
| `LaserOn[]` | 该时间片激光是否开启（true=走轮廓，false=空程快移） |
| `SampleRate` | 采样率 Hz；`Dt = 1/SampleRate` |
| `Count` / `Duration` | 采样点数 / 总时长（`Count × Dt`） |

关键换算：**每个时间片的位移步距** = 速度 × 时间片 = `speed × dt`。

- 进给步距 `feedStep = feedSpeed × dt`
- 快移步距 `rapidStep = rapidSpeed × dt`

---

## 三、Sample 主流程

```csharp
var ordered = OrderByNearest(polylines);     // ① 贪心最近邻排序，缩短空程
double dt = 1.0 / sampleRate;
Vec2 cur = ordered.Count > 0 ? ordered[0].Points[0] : Vec2.Zero;
double residual = 0;                          // ② 跨段相位缓冲（关键！）

foreach (var pl in ordered)
{
    var pts = new List<Vec2>(pl.Points);
    if (pl.Closed && pts.Count > 1 && !pts[0].Equals(pts[^1]))
        pts.Add(pts[0]);                      // ③ 闭合轮廓补一个回到起点的收尾段

    // ④ 空程：当前位置 → 本轮廓起点（激光关，rapidStep 步距）
    residual = InterpolateSegment(cur, pts[0], rapidSpeed * dt, residual, xs, ys, laser, false);
    cur = pts[0];

    // ⑤ 轮廓段：逐段插补（激光开，feedStep 步距）
    for (int i = 1; i < pts.Count; i++)
    {
        residual = InterpolateSegment(cur, pts[i], feedSpeed * dt, residual, xs, ys, laser, true);
        cur = pts[i];
    }
}
// ⑥ 末尾补一个终点采样，确保最后一点被记录
if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
{ xs.Add(cur.X); ys.Add(cur.Y); laser.Add(false); }
```

### 各步骤要点

- **① 最近邻排序**：`OrderByNearest` 把轮廓重排，让「上一条终点」尽量靠近「下一条起点」，减少激光关闭的空程时间；轮廓数 >5000 时自动切换网格加速版 `OrderByGrid`（复杂度近似 O(K)）。开折线还可整体反向以就近衔接。
- **② residual（跨段相位缓冲）**：**整个方法的灵魂**，详见第四节。它保证采样点在「跨越折线顶点」时仍保持严格等间距，不会在每个拐角处「重新起步」而产生密点。
- **③ 闭合补段**：闭合轮廓（如矩形、圆）若首尾点不重合，补一段回到起点，保证轮廓真正闭合加工。
- **④/⑤ 空程 vs 轮廓**：区别仅在**步距**（rapidStep vs feedStep）与**激光状态**（false vs true）。两者都用同一个 `InterpolateSegment`。
- **⑥ 收尾采样**：循环按步距推进，最后一个几何顶点通常落在两个采样步之间，`InterpolateSegment` 不会精确命中它；这里显式补记终点，避免末端丢失。

---

## 四、InterpolateSegment 深度剖析

这是 `Sample` 的核心子程序，负责「沿一条线段以固定步距撒点，并把用不完的相位传递给下一段」。

### 4.1 完整代码

```csharp
/// <summary>沿线段以固定步距采样，返回跨段剩余相位</summary>
private static double InterpolateSegment(Vec2 a, Vec2 b, double step, double residual,
    List<double> xs, List<double> ys, List<bool> laser, bool laserOn)
{
    double len = a.DistanceTo(b);
    if (len < 1e-12) return residual;      // ① 零长线段：直接透传 residual
    double s = step - residual;            // ② 本段第一个采样点的弧长位置
    while (s <= len)                       // ③ 沿线段等距撒点
    {
        double t = s / len;
        xs.Add(a.X + (b.X - a.X) * t);     //    线性插值得到落点
        ys.Add(a.Y + (b.Y - a.Y) * t);
        laser.Add(laserOn);
        s += step;
    }
    return step - (s - len);               // ④ 计算并返回新的剩余相位
}
```

### 4.2 参数与返回值

| 名称 | 含义 |
|------|------|
| `a`, `b` | 线段起点、终点 |
| `step` | 采样步距（= 速度 × dt），即相邻采样点应有的固定弧长间隔 |
| `residual` | **上一段结束时「距离下一个采样点还差多少弧长」**（入参） |
| 返回值 | **本段结束时新的 residual**，传给下一段 |
| `xs/ys/laser` | 输出缓冲，就地追加采样点 |
| `laserOn` | 本段激光状态 |

### 4.3 逐行解释

**① 零长线段保护**：两点重合（`len < 1e-12`）时无法定义方向，直接把 residual 原样返回，不撒点也不破坏相位。

**② 计算本段第一个采样点位置 `s = step − residual`**——这是理解 residual 的钥匙：

- `residual` 表示「上一段末尾，已经走过但还没凑够一个完整 step 的那段余量」。
- 因此本段的第一个采样点，不应从 `s = step` 开始（那会重复计入余量），而应提前 `residual`，即从 `s = step − residual` 开始。
- 举例：step=10，上一段结束时 residual=3（意思是上个采样点距上一段终点还剩 3，下一个整点应在再走 7 之后）。本段第一点位置 = 10−3 = 7 ✓，正好补齐那 3+7=10 的完整间隔。

**③ 等距撒点循环**：`s` 从首点位置起，每次 `+= step`，只要 `s ≤ len` 就落一个点。用参数 `t = s/len` 做线性插值 `a + (b−a)·t`，因此点严格落在线段上、且沿线段等弧长分布。

**④ 计算返回的新 residual**：循环结束时 `s > len`，说明「下一个本应撒的点」超出了本段终点，超出量为 `s − len`。那么距离该点还需再走的弧长 = `step − (s − len)`，这正是**留给下一段的 residual**。

### 4.4 residual 机制的意义（图示）

设 step = 4，一条折线由 A→B（长 6）→C（长 5）组成：

```
段 AB（len=6，入 residual=0）：
  s=4 撒点①    s=8>6 停 → 出 residual = 4-(8-6)=2
段 BC（len=5，入 residual=2）：
  s=4-2=2 撒点②   s=6>5 停 → 出 residual = 4-(6-5)=3
落点弧长位置：4, 6, 10 …  ← 全局间隔恒为 4，跨越顶点 B 处无异常
```

**若没有 residual**（每段都从 s=step 重新起步），则每个顶点后都会「重新计时」，导致：
- 顶点附近采样点忽疏忽密，等时性被破坏；
- 频率分解看到的不是等间隔序列，滤波结果失真；
- 短线段（len < step）会被整段跳过而**完全无采样点**。

有了 residual，短线段虽自身不撒点，但其长度被累加进相位，**不会丢失任何路径长度**，落点在全局上始终严格等间距。

---

## 五、尖锐拐角会不会被丢失？（重要评估）

> 用户提问：两个采样点之间恰好夹着一个尖角，等时采样会不会把这个尖角丢掉？

**结论：会「削角」，但不会「丢路径」，且误差有严格上界。** 需要区分两个概念：

### 5.1 顶点不是强制采样点

`InterpolateSegment` 只在 `s = step−residual, 2·step−residual, …` 这些**等弧长位置**撒点，**不会**把线段终点（即折线顶点 `b`）强制作为采样点；`Sample` 末尾的「补记终点」也只补整条路径最后一个点，不补中间顶点。

因此：当顶点 B 恰好落在两个相邻采样点 `P_k`（在段 AB 上，B 之前）与 `P_{k+1}`（在段 BC 上，B 之后）之间时，**顶点 B 本身不会出现在采样序列里**。伺服按采样点行走时，会用一条弦 `P_k → P_{k+1}` 直接跨过去，把尖角**削平一点**。

```
真实路径：  P_k ──→ B(尖角) ──→ P_{k+1}
采样重建：  P_k ─────────────→ P_{k+1}   （弦，绕过 B）
            └── 偏差 h ──┘
```

注意：`P_k`、`P_{k+1}` 本身是**精确落在段 AB、BC 上的**（线性插值 `t∈(0,1]`），几何无误差；误差只发生在「两采样点之间的重建」，即被削掉的那一小块尖角。

> ✅ **已实现修正**：`Sample` 现已内置「顶点吸附」（参数 `cornerAngleDeg`，默认 150°）。内角小于阈值的**尖角顶点会被强制作为采样点**，不再被弦切；仅对接近直线（内角 ≥ 阈值，如圆弧密集分段）的顶点保持纯等时采样。详见 §5.5。

### 5.2 削角误差的严格上界

设采样步距 `step = feedSpeed / sampleRate`，顶点处两段的**夹角为 θ**（θ=180° 为直线无拐角，θ→0° 为极尖的针状角）。设 B 之前的采样点距 B 弧长 `d₁`、之后距 B 弧长 `d₂`，因两点相邻故 `d₁ + d₂ = step`。

顶点 B 到弦 `P_k P_{k+1}` 的距离（即削掉的高度 / sagitta）：

```
h = d₁·d₂·sinθ / √(d₁² + d₂² − 2·d₁·d₂·cosθ)
```

对 `d₁+d₂=step` 求极值，**最坏情况在 d₁=d₂=step/2（顶点恰在两采样点正中）**：

```
h_max = (step/2)·cos(θ/2)   ≤   step/2
```

- θ=180°（直线）：cos90°=0 → h=0，无误差 ✓
- θ→0°（极尖针角）：cos0°=1 → h→step/2，达到上界 ✓

**关键结论：无论拐角多尖，单个尖角被削掉的最大偏差不超过 `step/2 = feedSpeed/(2·sampleRate)`。**

### 5.3 量化举例

| 进给速度 | 采样率 | step | 最坏削角误差 h_max |
|----------|--------|------|--------------------|
| 80 mm/s | 1000 Hz | 0.08 mm | ≤ 0.04 mm |
| 80 mm/s | 2000 Hz | 0.04 mm | ≤ 0.02 mm |
| 30 mm/s | 1000 Hz | 0.03 mm | ≤ 0.015 mm |

误差与 `step` 成正比，**提高采样率或降低进给速度即可任意压小**。

### 5.4 为什么这在本方案中通常可接受

采样得到的轨迹随后要过 `FrequencyDecomposer` 的**零相位低通滤波**：尖角是极高频特征，本就会被分频——高频残差交给振镜，而振镜自身带宽有限，**物理上无法走出数学意义的绝对尖角**（那需要无穷大加速度）。也就是说，即便把顶点 B 完整保留为采样点，最终联动系统仍会把它动态圆角化。采样阶段 `step/2` 量级的削角，通常**远小于**分频+伺服带宽带来的动态圆角，故一般可忽略。

### 5.5 尖角保真的工程实现（顶点吸附，已落地）

当工艺要求锐角保真时，本方案采用**顶点吸附**策略（另可辅以局部加密），已在 `PathSampler.Sample` 中实现：

**触发条件**——逐段插补到达某轮廓内部顶点后，计算该顶点处 `prev→vertex→next` 的内角 θ：

```csharp
// 内角小于阈值（cos 大于 cos(阈值)）即判为尖角
if (snapCorners && TryGetCornerNeighbor(pts, i, pl.Closed, out Vec2 nextPt)
    && IsSharpCorner(pts[i - 1], cur, nextPt, cornerCos))
{
    if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
    { xs.Add(cur.X); ys.Add(cur.Y); laser.Add(true); }   // 强制插入顶点
    residual = 0;   // 从尖角顶点重新计相位
}
```

**设计要点**：

1. **阈值 `cornerAngleDeg`（默认 150°）**：内角 < 150° 的顶点才吸附。圆弧密集分段的顶点内角接近 180°（如 170°）不会触发，避免无谓增点；真正的直角（90°）、锐角（<90°）则必被保留。设为 ≥180° 可关闭。
2. **重置 `residual = 0`**：从尖角顶点重新起相位，确保下一段从顶点后一个完整 `step` 处才撒第一点。
3. **闭合轮廓接缝**：`TryGetCornerNeighbor` 在末点回绕取 `pts[1]` 作为 next，使接缝处的角也能正确判定；开折线的自由末端无 next，不当作拐角。
4. **只作用于轮廓段**（`laserOn=true`），空程不受影响。

**代价与权衡**：被吸附的顶点前会出现一个略小于 `step` 的间隔（等时性在该点有极小扰动），但弦切误差归零。对后续零相位低通滤波而言，单点相位微扰动可忽略；保真度换取值得。

**补充手段**：若仍需更高保真，可在拐角邻域提高 `sampleRate` / 降低 `feedSpeed` 直接压小 `step`，与顶点吸附叠加使用。

---

## 六、正确性保证小结

| 保证项 | 机制 |
|--------|------|
| 采样严格等时/等距 | 固定 `step` + `residual` 跨段相位缓冲 |
| 不丢短线段路径 | 短段不撒点但长度累加进 residual |
| 落点精确落在几何上 | 线性插值 `a + (b−a)·t`，`t∈(0,1]` |
| 轮廓闭合完整 | 闭合轮廓补首尾衔接段 |
| 末端不丢失 | 循环后显式补记终点采样 |
| 激光时序正确 | 空程 `laserOn=false`、轮廓 `laserOn=true` |
| 空程最短 | 前置 `OrderByNearest`（大数据切换网格加速） |
| 尖角保真 | **顶点吸附**：内角 < `cornerAngleDeg`（默认 150°）的顶点强制作为采样点，弦切误差归零 |
| 非尖角不冗余增点 | 内角 ≥ 阈值（如圆弧密集分段）保持纯等时采样，不额外插点 |

---

## 七、示例

```csharp
// 一个 100mm 直线轮廓，进给 80mm/s，采样率 1000Hz
var line = new PathPolyline { Closed = false };
line.Points.Add(new Vec2(0, 0));
line.Points.Add(new Vec2(100, 0));

var traj = PathSampler.Sample(new[] { line }, feedSpeed: 80, jumpSpeedPlatform: 500, jumpSpeedGalvo: 2000, sampleRate: 1000);
// feedStep = 80 * (1/1000) = 0.08 mm/点
// 轮廓段约 100 / 0.08 = 1250 个激光开采样点（+ 空程 + 收尾点）
Console.WriteLine($"点数={traj.Count}  时长={traj.Duration:F3}s");
```

---

## 附：一句话结论

> `Sample` 把折线集合重排后逐段以「速度×dt」为步距做等时插补；`InterpolateSegment` 用 **residual 跨段相位缓冲** 保证落点在整条路径上严格等间距、短段不丢失。对夹在两采样点之间的**尖锐拐角**，`Sample` 通过**顶点吸附**（`cornerAngleDeg` 默认 150°）强制保留尖角顶点，弦切误差归零；未触发时单尖角削角也不超过 `step/2`，且不丢路径长度——这是后续频率分解能正确工作的前提。
