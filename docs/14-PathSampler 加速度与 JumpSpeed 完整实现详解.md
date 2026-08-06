# 14 - PathSampler 加速度与 JumpSpeed 完整实现详解

## 一、概述

本次更新围绕运动动力学参数做了两件事，二者落地程度**不同**，务必区分：

1. ✅ **JumpSpeed 独立配置（已生效）**：平台和振镜空移速度分离，`Sample` 主流程按 `rapidSpeed = min(√(jumpSpeedPlatform² + jumpSpeedGalvo²), 1000)` 合成空移速度并参与采样。
2. ⚠️ **加速度三段式采样（已编写但未接入主流程）**：`CalculateAccelDecelDistances` 与 `InterpolateSegmentWithAccel` 两个方法已在 `PathSampler.cs` 中实现，但 `Sample` 主流程**当前并未调用**它们——空程段（L79）与轮廓段（L106）都仍走恒速的 `InterpolateSegment`。因此 `accelPlatform / accelGalvo / decelPlatform` 目前**不参与实际采样计算**，仅作为签名参数保留、等待后续接线。

> 阅读提示：本文第三、四节描述的三段式加减速逻辑是**已存在但尚未启用**的实现，代表设计意图与代码现状，而非当前采样管线的实际行为。若要判断某次采样是否变速，以 `Sample` 是否调用 `InterpolateSegmentWithAccel` 为准——当前答案是"否"。

---

## 二、核心改进

### 2.1 方法签名对比

**改进前**：
```csharp
public static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,          // 进给速度
    double rapidSpeed,         // 快移速度（单一值）
    double sampleRate,         // 采样频率
    double cornerAngleDeg = 150,
    double accel = 1000.0,     // 加速度（未使用）
    double cornerFactor = 0.5,
    double decel = 0)          // 减速度（未使用）
```

**改进后**：
```csharp
public static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,              // 进给速度
    double jumpSpeedPlatform,      // 平台空移速度 ⭐新增
    double jumpSpeedGalvo,         // 振镜空移速度 ⭐新增
    double sampleRate,             // 采样频率
    double cornerAngleDeg = 150,
    double accelPlatform = 1000.0, // 平台加速度 ⭐独立
    double accelGalvo = 5000.0,    // 振镜加速度 ⭐独立
    double cornerFactor = 0.5,
    double decelPlatform = 0)      // 平台减速度 ⭐独立
```

### 2.2 参数物理意义

| 参数 | 符号 | 默认值 | 单位 | 物理意义 |
|------|------|--------|------|---------|
| **feedSpeed** | V_feed | 80 | mm/s | 激光出光时的加工速度 |
| **jumpSpeedPlatform** | V_jump_platform | 500 | mm/s | 激光关闭时平台的快速移动速度 |
| **jumpSpeedGalvo** | V_jump_galvo | 2000 | mm/s | 激光关闭时振镜的快速扫描速度 |
| **accelPlatform** | a_platform | 1000 | mm/s² | 平台伺服电机的加速度 |
| **accelGalvo** | a_galvo | 5000 | mm/s² | 振镜电机的加速度 |
| **cornerFactor** | k_corner | 0.5 | - | 尖角处的速度衰减系数 |
| **decelPlatform** | d_platform | 1000 | mm/s² | 平台伺服电机的减速度 |

---

## 三、加速度三段式实现（已编写，未接入主流程）

> ⚠️ **状态说明**：本节代码对应 `PathSampler.CalculateAccelDecelDistances` 与 `InterpolateSegmentWithAccel`。它们已存在于源码中，但 `Sample` 主循环并未调用——当前采样仍是恒速。以下内容描述这两个方法的内部逻辑，供后续接线时参考，并非现网采样行为。

### 3.1 加速段/减速段计算

**核心公式**：
```
加速距离：s_accel = (v_target² - v_current²) / (2 × a)
减速距离：s_decel = (v_target² - v_final²) / (2 × d)

其中：
- v_current：当前速度
- v_target：目标速度
- v_final：最终速度（通常为 0）
- a：加速度
- d：减速度
```

**代码实现**：
```csharp
// 计算加速段和减速段长度
double accelDist = Math.Max(0, (targetSpeed * targetSpeed - currentSpeed * currentSpeed) / (2.0 * accel));
double decelDist = Math.Max(0, (targetSpeed * targetSpeed) / (2.0 * decel));  // 减速到 0

// 如果线段太短，无法完成加速和减速
if (len < accelDist + decelDist)
{
    // 按比例分配加速和减速
    double ratio = accel / (accel + decel);
    accelDist = len * ratio;
    decelDist = len * (1 - ratio);
}

double fullSpeedDist = len - accelDist - decelDist;
fullSpeedDist = Math.Max(0, fullSpeedDist);
```

### 3.2 三段式采样

**算法流程**：
```
1. 加速段：从 v_current 加速到 v_target
   - 速度线性增加
   - 采样步长逐渐增大
   
2. 匀速段：保持 v_target
   - 速度恒定
   - 采样步长固定
   
3. 减速段：从 v_target 减速到 0
   - 速度线性减小
   - 采样步长逐渐减小
```

**代码实现**：
```csharp
// 1. 加速段
while (s < accelDist && s < len)
{
    // 计算当前位置的速度（线性加速）
    double localSpeed = currentSpeed + (accel * s / Math.Max(accelDist, 1e-9));
    localSpeed = Math.Min(localSpeed, targetSpeed);
    
    double step = localSpeed * dt;
    s += step;
    
    if (s <= len)
    {
        double t = s / len;
        xs.Add(a.X + (b.X - a.X) * t);
        ys.Add(a.Y + (b.Y - a.Y) * t);
        laser.Add(laserOn);
    }
}

// 2. 匀速段
while (s < accelDist + fullSpeedDist && s < len)
{
    double step = targetSpeed * dt;
    s += step;
    // ... 采样逻辑
}

// 3. 减速段
while (s < len)
{
    double distInDecel = s - accelDist - fullSpeedDist;
    double localSpeed = targetSpeed - (decel * distInDecel / Math.Max(decelDist, 1e-9));
    localSpeed = Math.Max(localSpeed, 0);
    
    double step = localSpeed * dt;
    s += step;
    // ... 采样逻辑
}
```

### 3.3 效果对比

**改进前**（恒定速度采样）：
```
线段长度 = 100 mm
进给速度 = 100 mm/s
采样频率 = 1000 Hz
采样步长 = 0.1 mm（恒定）

采样点数 = 100 / 0.1 = 1000 点
```

**改进后**（变速采样）：
```
线段长度 = 100 mm
目标速度 = 100 mm/s
加速度 = 1000 mm/s²
减速度 = 1000 mm/s²

加速距离 = (100² - 0²) / (2 × 1000) = 5 mm
减速距离 = (100² - 0²) / (2 × 1000) = 5 mm
匀速距离 = 100 - 5 - 5 = 90 mm

加速段采样点 ≈ 50 点（步长从 0 逐渐增加到 0.1）
匀速段采样点 = 90 / 0.1 = 900 点
减速段采样点 ≈ 50 点（步长从 0.1 逐渐减小到 0）

总采样点数 ≈ 1000 点
但速度分布更符合实际物理过程
```

---

## 四、JumpSpeed 独立配置

### 4.1 合成空移速度

**物理模型**：
```
激光头空移速度 V_rapid = V_platform + V_galvo（向量和）

由于平台和振镜正交运动：
|V_rapid| = √(V_platform² + V_galvo²)
```

**代码实现**：
```csharp
// 计算合成空移速度（平台和振镜的向量和）
double rapidSpeed = Math.Sqrt(jumpSpeedPlatform * jumpSpeedPlatform + 
                              jumpSpeedGalvo * jumpSpeedGalvo);

// 限制最大速度，防止过快导致采样点过疏
rapidSpeed = Math.Min(rapidSpeed, 1000.0);  // 上限 1000 mm/s
```

> ⚠️ 注意 1000 mm/s 的上限：当 `jumpSpeedPlatform=500 / jumpSpeedGalvo=2000` 时，向量合成约 2062 mm/s，会被**钳到 1000 mm/s**。因此后文"合成速度"相关的示例数值仅代表钳制前的理论值，实际参与采样的 `rapidSpeed` 不超过 1000。

### 4.2 加速度选择

**策略**：使用较小的加速度作为保守估计

**原因**：
- 平台和振镜必须同时达到目标速度
- 加速度较小的轴会成为瓶颈
- 保守估计确保两者都能达到

**代码实现**：
```csharp
// 使用较小的加速度作为保守估计（确保两者都能达到）
double accel = Math.Min(accelPlatform, accelGalvo);
```

### 4.3 示例计算

**参数配置**：
```
jumpSpeedPlatform = 500 mm/s
jumpSpeedGalvo = 2000 mm/s
accelPlatform = 1000 mm/s²
accelGalvo = 5000 mm/s²
```

**计算结果**：
```
合成空移速度 = √(500² + 2000²) = √(250000 + 4000000) = √4250000 ≈ 2062 mm/s

合成加速度 = min(1000, 5000) = 1000 mm/s²（受限于平台）

加速到 2062 mm/s 所需时间：
t = v / a = 2062 / 1000 = 2.062 s

加速距离：
s = 0.5 × v² / a = 0.5 × 2062² / 1000 = 2126 mm
```

---

## 五、参数传递链路

### 5.1 UI → ViewModel → PathSampler

**MainViewModel 中的调用**：
```csharp
public void Decompose()
{
    if (AutoCutoff)
    {
        if (Polylines.Count > PathSampler.MaxSampleContours)
        {
            // 子集采样
            var subset = PathSampler.Decimate(Polylines, PathSampler.MaxSampleContours);
            var subsetTraj = PathSampler.Sample(subset, 
                FeedSpeed,           // UI 绑定
                JumpSpeedPlatform,   // UI 绑定 ⭐新增
                JumpSpeedGalvo,      // UI 绑定 ⭐新增
                SampleRate,          // UI 绑定
                cornerAngleDeg: 150,
                accelPlatform: AccelPlatform,  // UI 绑定 ⭐独立
                accelGalvo: AccelGalvo,        // UI 绑定 ⭐独立
                cornerFactor: CornerFactor);   // UI 绑定
            
            double cutoff = FrequencyDecomposer.DecomposeAuto(subsetTraj, GalvoFov).CutoffHz;
            
            // 全量采样
            var traj = PathSampler.Sample(Polylines, 
                FeedSpeed, JumpSpeedPlatform, JumpSpeedGalvo, SampleRate,
                cornerAngleDeg: 150,
                accelPlatform: AccelPlatform,
                accelGalvo: AccelGalvo,
                cornerFactor: CornerFactor);
            
            Plan = FrequencyDecomposer.Decompose(traj, cutoff, GalvoFov);
        }
        // ...
    }
}
```

### 5.2 参数传递流程

```
UI Slider (JumpSpeedPlatform, JumpSpeedGalvo, AccelPlatform, AccelGalvo)
    ↓ (WPF Binding)
MainViewModel (属性)
    ↓ (方法参数)
PathSampler.Sample(..., jumpSpeedPlatform, jumpSpeedGalvo, accelPlatform, accelGalvo, ...)
    ↓ (内部计算)
rapidSpeed = min(√(jumpSpeedPlatform² + jumpSpeedGalvo²), 1000)   ← 已生效，参与空程采样
    ↓
InterpolateSegment(...)   ← 恒速插补（空程与轮廓段均走此路径）

// accelPlatform / accelGalvo / decelPlatform 已随参数传入，但主流程当前未消费；
// InterpolateSegmentWithAccel（三段式加减速）已实现但未被调用。
```

---

## 六、效果对比与验证

### 6.1 加速度效果（三段式接入后的预期）

> ⚠️ 以下"改进后"数据是三段式加减速**接入主流程后**的预期效果，当前采样仍为恒速，尚未产生此效果。

**测试场景**：
```
轮廓：正方形 100mm × 100mm
进给速度：100 mm/s
加速度：1000 mm/s²
采样频率：1000 Hz
```

**改进前**（恒定速度）：
```
尖角处速度 = 100 mm/s（不减速）
尖角保真度：⭐（完全圆角）
过烧风险：⭐⭐⭐⭐⭐
```

**改进后**（变速采样）：
```
加速段：0-5mm，速度从 0 加速到 100 mm/s
匀速段：5-95mm，速度保持 100 mm/s
减速段：95-100mm，速度从 100 mm/s 减速到 0
尖角保真度：⭐⭐⭐⭐⭐（速度降至 0）
过烧风险：⭐（停留时间短）
```

### 6.2 JumpSpeed 独立配置效果

**参数配置**：
```
场景 1：jumpSpeedPlatform = 300 mm/s, jumpSpeedGalvo = 1000 mm/s
场景 2：jumpSpeedPlatform = 500 mm/s, jumpSpeedGalvo = 2000 mm/s
场景 3：jumpSpeedPlatform = 800 mm/s, jumpSpeedGalvo = 4000 mm/s
```

**合成速度对比**：
| 场景 | V_platform | V_galvo | V_rapid | 空程时间减少 |
|------|-----------|---------|---------|-----------|
| 1 | 300 mm/s | 1000 mm/s | 1044 mm/s | 基准 |
| 2 | 500 mm/s | 2000 mm/s | 2062 mm/s | -49.6% |
| 3 | 800 mm/s | 4000 mm/s | 4079 mm/s | -60.9% |

**结论**：
- ✅ 提高 JumpSpeed 可显著减少空程时间
- ✅ 振镜速度对合成速度影响更大（因为数值更大）
- ⚠️ 需平衡精度和效率

---

## 七、使用建议

### 7.1 参数配置策略

**精密加工模式**：
```csharp
feedSpeed = 80 mm/s
jumpSpeedPlatform = 300 mm/s
jumpSpeedGalvo = 1500 mm/s
accelPlatform = 800 mm/s²
accelGalvo = 3000 mm/s²
cornerFactor = 0.2
```

**通用加工模式**（推荐）：
```csharp
feedSpeed = 100 mm/s
jumpSpeedPlatform = 500 mm/s
jumpSpeedGalvo = 2000 mm/s
accelPlatform = 1000 mm/s²
accelGalvo = 5000 mm/s²
cornerFactor = 0.5
```

**高效生产模式**：
```csharp
feedSpeed = 150 mm/s
jumpSpeedPlatform = 800 mm/s
jumpSpeedGalvo = 4000 mm/s
accelPlatform = 2000 mm/s²
accelGalvo = 8000 mm/s²
cornerFactor = 0.8
```

### 7.2 参数调整流程

1. **确定加工目标**：精度优先 vs 效率优先
2. **配置 JumpSpeed**：
   - 精度优先 → 低值（300/1500）
   - 平衡 → 中值（500/2000）
   - 效率优先 → 高值（800/4000）
3. **配置加速度**：
   - 根据设备能力
   - 通常 accelGalvo = 3-5 × accelPlatform
4. **验证效果**：
   - 试切测试件
   - 测量尖角保真度
   - 评估加工时间

---

## 八、常见问题

### Q1：为什么合成速度用向量合成而不是简单相加？

**A**：
- 平台和振镜是**正交运动**（X-Y 方向）
- 速度是矢量，必须用向量合成
- `V_rapid = √(V_platform² + V_galvo²)`
- 简单相加会高估实际速度

### Q2：为什么加速度取较小值？

**A**：
- 平台和振镜必须**同时达到目标速度**
- 加速度较小的轴会成为瓶颈
- 保守估计确保两者都能达到
- 否则会出现一轴已完成加速，另一轴还在加速的情况

### Q3：加速度参数目前用上了吗？

**A**：
- **暂时没有。** `accelPlatform / accelGalvo / decelPlatform` 会随参数传入 `Sample`，但主流程尚未消费它们。
- 三段式采样逻辑已写在 `InterpolateSegmentWithAccel` / `CalculateAccelDecelDistances` 中，但没有任何调用点——`Sample` 的空程段与轮廓段都走恒速的 `InterpolateSegment`。
- 待后续把 `InterpolateSegmentWithAccel` 接入主循环后，加速段/匀速段/减速段与动态步长才会真正生效。

### Q4：如何优化空程时间？

**A**：
1. **提高 JumpSpeed**：
   - 优先提高振镜速度（影响更大）
   - 适当提高平台速度
2. **提高加速度**：
   - 减少加速段时间
   - 增加匀速段比例
3. **优化路径规划**：
   - 减少空程距离
   - 使用贪心最近邻排序

---

## 九、总结

### 9.1 核心改进

1. ✅ **JumpSpeed 独立配置（已生效）**：
   - 平台和振镜空移速度分离
   - 向量合成计算 rapidSpeed，并钳制到 1000 mm/s 上限
   - 参与 `Sample` 空程段采样

2. ⚠️ **加速度三段式采样（已编写，未接入）**：
   - `CalculateAccelDecelDistances` / `InterpolateSegmentWithAccel` 已实现
   - 但 `Sample` 主流程尚未调用，`accel*/decelPlatform` 当前不参与采样
   - 待接线后才产生加速→匀速→减速的动态步长

### 9.2 物理意义

- **feedSpeed**：激光头合成速度（平台 + 振镜）
- **jumpSpeedPlatform**：平台空移速度（激光关闭）
- **jumpSpeedGalvo**：振镜空移速度（激光关闭）
- **accelPlatform**：平台加速度（影响加速段长度）
- **accelGalvo**：振镜加速度（影响加速段长度）
- **cornerFactor**：尖角速度衰减系数

### 9.3 下一步优化方向

1. ⬜ **把三段式加减速接入 `Sample` 主流程**（当前最关键的缺口）
2. ⬜ **实时监控**：显示实际速度和加速度曲线
3. ⬜ **自适应参数**：根据材料/厚度自动推荐参数
4. ⬜ **参数学习**：基于质量数据自动优化参数

---

**文档版本**：v1.0  
**最后更新**：2026-07-29  
**维护者**：GalvoStage 开发团队
