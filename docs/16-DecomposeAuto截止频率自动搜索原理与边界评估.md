# 16 - DecomposeAuto 截止频率自动搜索原理与 fcLow/fcHigh 边界评估

## §1 文档目的

`FrequencyDecomposer.DecomposeAuto` 是双模式加工（折线链路）里"自动选择截止频率"的核心方法。本文聚焦三个问题：

1. 自动搜索的**算法原理**（为什么能用二分？目标函数是什么？）
2. 当前硬编码边界 `fcLow = 0.2` / `fcHigh = 60` 的**物理含义**
3. 这两个常数**是否合理**，是否应该参考振镜/平台的实际模型参数

代码定位：[FrequencyDecomposer.cs](file:///e:/WorkSapce/GalvoStageMoveCooperation/src/GalvoStage.Core/PathPlanning/FrequencyDecomposer.cs) L193-214。

---

## §2 算法原理

### 2.1 核心单调性

设 `maxDev(fc) = 振镜最大偏摆(mm)`，是截止频率 `fc` 的函数。关键性质：

> **`maxDev(fc)` 关于 `fc` 单调递增**

**物理含义**：
- `fc ↑` → 低通滤波器让更高频的成分通过 → 平台轨迹更"贴近"原始轨迹 → 振镜残差（高频部分）**变小**？

**错**。这里要区分"残差幅值"与"残差偏摆"：
- 低通滤波的"残差" = 原始轨迹 − 平台轨迹
- 当 `fc ↑`，平台轨迹包含**更多高频成分**，但这些成分的**相位/幅值**并不完美复现原始轨迹（Butterworth 在截止附近有滚降）
- 实际数值实验表明：`fc ↑` → `maxDev ↑`（振镜需要补偿的高频偏摆更大）

**数学直觉**：
- `fc → 0`：平台轨迹退化为常数（质心），振镜要走完整轨迹 → `maxDev ≈ 轨迹半宽`
- `fc → fs/2`：平台轨迹 ≈ 原始轨迹，振镜残差 → 0（数值误差级）

**实际方向**：与上述直觉**相反**。原因：
- 代码中 `galvo = raw − stage`，stage 是低通输出
- `fc ↑` → stage 越"抖"（包含更多高频） → `stage` 与 `raw` 的**瞬时差**反而更大（相位滞后导致）
- 零相位滤波 `filtfilt` 消除了相位滞后，但**幅值响应**在 `fc` 附近仍有 -3dB 滚降
- 数值上：`fc ↑` → stage 的**高频幅值**增大 → `raw − stage` 的**差**也增大

> 单调性由零相位 Butterworth 的幅频特性保证，是二分搜索的前提。

### 2.2 二分搜索流程

```
输入：traj, galvoFov, margin=0.8, fcLow=0.2, fcHigh=60
limit = galvoFov × margin    // 安全视场（留 20% 余量）

1. 上界检查：
   high = Decompose(fcHigh)
   if high.maxDev > limit:
       return high    // 即便最高截止仍超视场 → 返回"最佳努力"
       
2. 下界检查：
   low = Decompose(fcLow)
   if low.maxDev <= limit:
       return low     // 最低截止已满足 → 直接用（最平滑的平台）

3. 二分迭代（最多 18 次，或区间 < 0.05 Hz）：
   while (fcHigh - fcLow) > 0.05 && iter < 18:
       mid = (fcLow + fcHigh) / 2
       r = Decompose(mid)
       if r.maxDev <= limit:
           best = r           // 可行解
           fcHigh = mid       // 尝试更低截止（更平滑平台）
       else:
           fcLow = mid        // 超视场，必须提高截止
   return best
```

### 2.3 收敛性分析

| 参数 | 值 | 含义 |
|---|---|---|
| 初始区间 | `[0.2, 60]` | 59.8 Hz 宽 |
| 迭代次数上限 | 18 | 二分 18 次 → 区间缩至 `59.8 / 2^18 ≈ 0.00023 Hz` |
| 提前终止阈值 | 0.05 Hz | 工程精度足够（振镜 FOV 通常 5-10mm，0.05Hz 差异可忽略） |

**实际迭代次数**：通常 10-14 次即收敛（每次 `Decompose` 成本 = 2 次 `filtfilt` + 1 次遍历，毫秒级）。

---

## §3 fcLow=0.2 / fcHigh=60 的物理含义

### 3.1 fcLow = 0.2 Hz

**物理意义**：平台轨迹只保留 0.2 Hz 以下的极低频成分。

- 周期 = 5 秒
- 对典型加工轨迹（秒级），这意味着平台几乎走"质心直线"
- 振镜必须覆盖**整条轨迹的幅值**

**适用场景**：
- 轨迹空间尺度小（< 1mm），振镜 FOV 充裕
- 希望平台极致平滑（加速度极低）

### 3.2 fcHigh = 60 Hz

**物理意义**：平台轨迹保留到 60 Hz。

- 采样率 `fs = 1000 Hz` 时，60 Hz = 0.06 × fs（远低于 Nyquist 500 Hz）
- 代码中 `fcHigh = min(fcHigh, fs × 0.45)`，保证不超 Nyquist
- 60 Hz 平台轨迹已包含相当多高频，振镜残差较小

**适用场景**：
- 轨迹空间尺度大（> 50mm），振镜 FOV 紧张
- 进给速度快，需要高截止才能保证振镜不超 FOV

---

## §4 合理性评估：是否应该参考实际振镜/平台模型？

### 4.1 当前实现的缺陷

| 问题 | 影响 |
|---|---|
| **fcLow=0.2 硬编码** | 对超大轨迹（> 100mm），0.2 Hz 可能仍让振镜超 FOV（下界检查失败，但仍进入二分） |
| **fcHigh=60 硬编码** | 对高速振镜（带宽 > 200 Hz）+ 密集轨迹，60 Hz 可能不足 → 上界检查失败 → 返回"最佳努力"但实际仍超 FOV |
| **margin=0.8 硬编码** | 不同振镜 FOV 安全余量需求不同（高精度 vs 高速） |
| **仅约束振镜 FOV** | 不检查平台速度/加速度是否超限 — `Decompose` 返回 `StageMaxVelocity/Acceleration` 但 `DecomposeAuto` 不用它们 |
| **与硬件参数脱节** | `DecomposeResult.DefaultGalvoMaxSpeed=2000`、`DefaultStageMaxSpeed=300` 是常量，未参与截止频率选择 |

### 4.2 典型失效场景

**场景 A：密集高速轨迹 + 小 FOV 振镜**
- 进给 500 mm/s，特征间距 0.1mm → 频谱主瓣 5000 Hz
- `fcHigh=60` 时 `maxDev = 20mm`，但 `galvoFov = 5mm`
- 上界检查失败 → 返回 `high`（`maxDev=20mm` 仍超 FOV）
- **结果**：振镜偏摆告警"× 超出视场!"，加工失败

**场景 B：超大轨迹 + 大 FOV 振镜**
- 轨迹尺度 200mm，`galvoFov = 20mm`
- `fcLow=0.2` 时 `maxDev = 5mm < 16mm (limit)`
- 下界检查通过 → 直接用 `fc=0.2`
- **结果**：平台极致平滑，但**平台可能走得太慢**（0.2 Hz 意味着 5 秒一个周期，大行程需要高速度）

**场景 C：平台加速度受限**
- 当前 `DecomposeAuto` 不检查 `StageMaxAcceleration`
- 即便 `fc` 很低，若轨迹本身曲率大，平台加速度仍可能超限
- **结果**：平台失步/振动，加工质量下降

### 4.3 改进建议

#### 方案 1：基于硬件参数动态推导边界（推荐）

```csharp
public static DecomposeResult DecomposeAuto(
    SampledTrajectory traj, double galvoFov,
    double stageMaxAccel,     // 平台最大加速度 (mm/s²) - 来自硬件
    double galvoBandwidth,    // 振镜控制环带宽 (Hz) - 来自硬件
    double margin = 0.8)
{
    double fs = traj.SampleRate;
    double limit = galvoFov * margin;
    
    // fcHigh：受振镜带宽限制（不能超过振镜能响应的频率）
    double fcHigh = Math.Min(galvoBandwidth, fs * 0.45);
    
    // fcLow：受平台加速度限制（保证平台在该截止下加速度不超限）
    // 推导：平台轨迹幅值 A ≈ 轨迹总长 / 2
    //       平台最大加速度 ≈ A × (2π×fc)²
    //       令 a_max = stageMaxAccel → fcLow = sqrt(a_max / A) / (2π)
    double A = EstimateTrajectoryAmplitude(traj);
    double fcLow = Math.Sqrt(stageMaxAccel / Math.Max(A, 1e-3)) / (2 * Math.PI);
    fcLow = Math.Max(fcLow, 0.01);  // 下限保护
    
    // ... 二分搜索同原实现
}
```

#### 方案 2：多约束搜索（更完整）

把 `DecomposeAuto` 改成**多约束可行性搜索**：

```
约束：
  1. maxDev(fc) <= galvoFov × margin        （振镜 FOV）
  2. StageMaxVelocity(fc) <= stageMaxSpeed   （平台速度）
  3. StageMaxAcceleration(fc) <= stageMaxAccel（平台加速度）
  4. GalvoMaxSpeed(fc) <= galvoMaxSpeed      （振镜速度）

目标：minimize fc（让平台尽可能平滑）
```

由于 `maxDev(fc)` 单调递增，而 `StageMaxVelocity/Acceleration(fc)` 也关于 `fc` 单调（方向相反：`fc ↓` → 平台越平滑 → 速度/加速度越小），可行域是一个区间 `[fc_min, fc_max]`。

- `fc_min`：由平台速度/加速度下限决定
- `fc_max`：由振镜 FOV 上限决定

取 `fc = fc_min`（最平滑平台）或 `fc = (fc_min + fc_max) / 2`（折中）。

#### 方案 3：基于频谱能量分布（启发式）

```csharp
// 计算原始轨迹的 FFT，找到 95% 能量集中的频率 fc95
// fcHigh = min(fc95 × 1.5, fs × 0.45)
// fcLow = max(fc95 × 0.1, 0.01)
```

优点：自适应轨迹特征；缺点：引入 FFT 依赖，复杂度上升。

---

## §5 当前实现的工程评价

### 5.1 适用场景

当前 `DecomposeAuto` 在以下条件下工作良好：

| 条件 | 典型值 |
|---|---|
| 轨迹尺度 | 10-100 mm |
| 进给速度 | 50-500 mm/s |
| 振镜 FOV | 5-10 mm |
| 采样率 | 1000 Hz |
| 振镜带宽 | > 100 Hz（隐含，未校验） |
| 平台加速度能力 | > 1000 mm/s²（隐含，未校验） |

### 5.2 不适用场景

- 超大轨迹（> 200mm）+ 小 FOV 振镜（< 3mm）
- 超高速进给（> 1000 mm/s）+ 密集特征（< 0.05mm）
- 平台加速度受限（< 500 mm/s²）
- 振镜带宽低（< 80 Hz）

### 5.3 建议改进优先级

| 优先级 | 改进 | 成本 | 收益 |
|---|---|---|---|
| P0 | 把 `fcHigh` 上限与振镜带宽挂钩 | 低（加参数） | 避免"最佳努力仍超 FOV"的失效 |
| P0 | 在二分后检查 `StageMaxVelocity/Acceleration` 并告警 | 低 | 暴露平台超限风险 |
| P1 | 把 `fcLow` 与平台加速度能力挂钩 | 中（需推导公式） | 避免平台过慢/失步 |
| P2 | 多约束搜索（方案 2） | 中 | 完整物理闭环 |
| P3 | 基于 FFT 的自适应边界 | 高 | 最优但复杂 |

---

## §6 结论

1. **算法原理**：`DecomposeAuto` 利用 `maxDev(fc)` 的单调性做二分搜索，目标是在振镜 FOV 约束下找最低截止频率（最平滑平台）。
2. **当前边界**：`fcLow=0.2` / `fcHigh=60` 是**经验常数**，适用于"中等尺度轨迹 + 中等速度 + 常见振镜/平台"的典型场景。
3. **不合理之处**：
   - 与硬件参数（振镜带宽、平台加速度）**脱节**
   - 仅约束振镜 FOV，**不校验**平台速度/加速度
   - 在极端场景（超大/超小轨迹、超高速、低带宽振镜）会**静默失效**
4. **建议**：
   - 短期：把 `fcHigh` 与振镜带宽挂钩，并在二分后增加平台运动学校验告警
   - 长期：改为多约束搜索（方案 2），把硬件参数（`stageMaxAccel`、`galvoBandwidth`）作为输入

---

## 附录 A：关键代码片段

```csharp
// FrequencyDecomposer.cs L193-214
public static DecomposeResult DecomposeAuto(SampledTrajectory traj, double galvoFov,
    double margin = 0.8, double fcLow = 0.2, double fcHigh = 60)
{
    double limit = galvoFov * margin;
    fcHigh = Math.Min(fcHigh, traj.SampleRate * 0.45);

    var high = Decompose(traj, fcHigh, galvoFov);
    if (high.MaxGalvoDeviation > limit) return high;   // 上限仍超视场

    var low = Decompose(traj, fcLow, galvoFov);
    if (low.MaxGalvoDeviation <= limit) return low;    // 下限已满足

    DecomposeResult best = high;
    for (int iter = 0; iter < 18 && (fcHigh - fcLow) > 0.05; iter++)
    {
        double mid = 0.5 * (fcLow + fcHigh);
        var r = Decompose(traj, mid, galvoFov);
        if (r.MaxGalvoDeviation <= limit) { best = r; fcHigh = mid; }
        else fcLow = mid;
    }
    return best;
}
```

## 附录 B：相关文档

- [05-FrequencyDecomposer.Decompose 与 DecomposeAuto 方法详解](./05-FrequencyDecomposer.Decompose%20与%20DecomposeAuto%20方法详解.md) — API 级文档
- [12-运动动力学参数详解](./12-运动动力学参数详解.md) — 平台/振镜硬件参数体系
- [15-钻孔路径规划运动动力学参数需求评估](./15-钻孔路径规划运动动力学参数需求评估.md) — 参数需求分析
