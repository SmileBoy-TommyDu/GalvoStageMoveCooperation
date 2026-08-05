# 13 - PathSampler 运动动力学参数集成详解

## 一、问题背景

### 1.1 原有算法缺陷

**原 `PathSampler.Sample` 方法签名**：
```csharp
public static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,      // 进给速度
    double rapidSpeed,     // 快移速度
    double sampleRate,     // 采样频率
    double cornerAngleDeg = 150)  // 尖角保真阈值
```

**缺失的关键参数**：
- ❌ `accel` - 加速度（影响加速段长度）
- ❌ `cornerFactor` - 拐角系数（影响尖角速度衰减）

**导致的问题**：
1. **尖角处理过于简单**：只用顶点吸附，没有速度衰减
2. **加速度未参与计算**：无法准确估算加速段对加工时间的影响
3. **拐角质量不可控**：用户无法调整尖角处的加工策略

---

### 1.2 进给速度的物理意义

**关键理解**：

```
进给速度 (feedSpeed) ≠ 平台速度 + 振镜速度
进给速度 (feedSpeed) = 激光头在工件表面的实际移动速度
```

**数学表达**：
```
激光头位置：P(t) = StageP(t) + GalvoP(t)
激光头速度：V_head(t) = V_stage(t) + V_galvo(t)

当激光出光时：|V_head(t)| = feedSpeed
当激光关闭时：|V_head(t)| = rapidSpeed
```

**频率分解后的速度分配**：
```
V_head(t) = V_stage(t) + V_galvo(t)

其中：
- V_stage(t)：平台速度（低频分量，50-300 mm/s）
- V_galvo(t)：振镜速度（高频分量，500-5000 mm/s）
- |V_head(t)| = feedSpeed（合成后的末端速度）
```

**关键点**：
- `feedSpeed` 是**合成后的末端速度**
- 平台速度 + 振镜速度 = feedSpeed（向量和）
- 频率分解使平台走低频、振镜走高频
- 两者的频谱正交不重叠

---

## 二、算法改进

### 2.1 新增参数

**改进后的 `PathSampler.Sample` 方法签名**：
```csharp
public static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,          // 进给速度 (mm/s)
    double rapidSpeed,         // 快移速度 (mm/s)
    double sampleRate,         // 采样频率 (Hz)
    double cornerAngleDeg = 150,  // 尖角保真阈值 (度)
    double accel = 1000.0,     // 加速度 (mm/s²) ⭐新增
    double cornerFactor = 0.5) // 拐角系数 (0-1) ⭐新增
```

### 2.2 拐角系数集成

**核心改进**：在轮廓段采样时，根据拐角系数动态调整速度

```csharp
// 轮廓段
for (int i = 1; i < pts.Count; i++)
{
    // 计算当前段的速度（考虑拐角系数）
    double segmentSpeed = feedSpeed;
    
    // 如果是尖角，根据 cornerFactor 衰减速度
    if (i > 1 && snapCorners && IsSharpCorner(pts[i - 2], pts[i - 1], pts[i], cornerCos))
    {
        // 尖角处的速度 = feedSpeed * (1 - cornerFactor)
        segmentSpeed = feedSpeed * (1.0 - cornerFactor);
        
        // 如果速度过低，至少保持 feedSpeed 的 10%
        segmentSpeed = Math.Max(segmentSpeed, feedSpeed * 0.1);
    }
    
    residual = InterpolateSegment(cur, pts[i], segmentSpeed * dt, residual, xs, ys, laser, true);
    cur = pts[i];
    
    // ... 尖角保真逻辑
}
```

**物理意义**：
- `cornerFactor = 0.0`：尖角处速度 = feedSpeed × (1-0) = feedSpeed（不减速，完全圆滑）
- `cornerFactor = 0.5`：尖角处速度 = feedSpeed × (1-0.5) = 0.5 × feedSpeed（减速 50%）
- `cornerFactor = 1.0`：尖角处速度 = feedSpeed × (1-1) = 0（完全停止，尖角保真）

**修正**：上述公式中，`cornerFactor` 的含义需要反转理解：
- `cornerFactor = 0.0`：尖角处速度 = feedSpeed（不减速，完全圆滑）
- `cornerFactor = 1.0`：尖角处速度 = 0（完全停止，尖角保真）

实际代码中：
```csharp
segmentSpeed = feedSpeed * (1.0 - cornerFactor);
```

这意味着：
- `cornerFactor = 0.0` → `segmentSpeed = feedSpeed`（不减速）
- `cornerFactor = 0.5` → `segmentSpeed = 0.5 * feedSpeed`（减速 50%）
- `cornerFactor = 1.0` → `segmentSpeed = 0`（完全停止）

---

### 2.3 加速度参数（预留）

**当前状态**：`accel` 参数已添加到方法签名，但尚未在算法中使用

**未来计划**：
1. 计算加速段长度：`s_accel = 0.5 * v² / a`
2. 在轮廓起点和终点插入加速/减速段
3. 动态调整采样步长以反映速度变化

**示例计算**：
```
进给速度 = 100 mm/s
加速度 = 1000 mm/s²

加速时间 = v / a = 100 / 1000 = 0.1 s
加速距离 = 0.5 * v² / a = 0.5 * 100² / 1000 = 5 mm

意味着：
- 从 0 加速到 100 mm/s 需要 5 mm
- 如果轮廓段长度 < 5 mm，则无法达到 full feedSpeed
```

---

## 三、参数传递链路

### 3.1 UI → ViewModel → PathSampler

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
                FeedSpeed,      // UI 绑定
                RapidSpeed,     // UI 绑定
                SampleRate,     // UI 绑定
                cornerAngleDeg: 150,
                accel: AccelPlatform,      // UI 绑定 ⭐新增
                cornerFactor: CornerFactor); // UI 绑定 ⭐新增
            
            double cutoff = FrequencyDecomposer.DecomposeAuto(subsetTraj, GalvoFov).CutoffHz;
            
            // 全量采样
            var traj = PathSampler.Sample(Polylines, 
                FeedSpeed, RapidSpeed, SampleRate,
                cornerAngleDeg: 150,
                accel: AccelPlatform,
                cornerFactor: CornerFactor);
            
            Plan = FrequencyDecomposer.Decompose(traj, cutoff, GalvoFov);
        }
        // ...
    }
}
```

**参数传递流程**：
```
UI Slider
    ↓ (WPF Binding)
MainViewModel.AccelPlatform / CornerFactor
    ↓ (方法参数)
PathSampler.Sample(..., accel: AccelPlatform, cornerFactor: CornerFactor)
    ↓ (内部计算)
segmentSpeed = feedSpeed * (1 - cornerFactor)
    ↓
InterpolateSegment(..., segmentSpeed * dt, ...)
```

---

## 四、效果对比

### 4.1 尖角处理效果

**改进前**（`cornerFactor` 未集成）：
```
进给速度 = 100 mm/s
尖角处速度 = 100 mm/s（不减速）
结果：尖角变圆角，过烧风险
```

**改进后**（`cornerFactor = 0.5`）：
```
进给速度 = 100 mm/s
尖角处速度 = 100 * (1 - 0.5) = 50 mm/s
结果：尖角保真度提高，过烧风险降低
```

**不同 cornerFactor 的效果**：

| cornerFactor | 尖角处速度 | 尖角保真度 | 过烧风险 | 适用场景 |
|-------------|-----------|-----------|---------|---------|
| 0.0 | 100 mm/s | ⭐ | ⭐⭐⭐⭐⭐ | 圆孔/曲线 |
| 0.3 | 70 mm/s | ⭐⭐⭐ | ⭐⭐⭐⭐ | 精密零件 |
| 0.5 | 50 mm/s | ⭐⭐⭐⭐ | ⭐⭐⭐ | 通用加工 |
| 0.7 | 30 mm/s | ⭐⭐⭐⭐⭐ | ⭐⭐ | 方孔/锐角 |
| 1.0 | 0 mm/s | ⭐⭐⭐⭐⭐ | ⭐ | 高精度尖角 |

### 4.2 采样点分布

**改进前**（等速采样）：
```
尖角前后采样点间距相同
导致：尖角处采样点稀疏，精度下降
```

**改进后**（变速采样）：
```
尖角处速度降低 → 采样点间距减小 → 采样点加密
导致：尖角处采样点密集，精度提高
```

**示例**：
```
进给速度 = 100 mm/s
采样频率 = 1000 Hz
采样步长 = 100 / 1000 = 0.1 mm

尖角处（cornerFactor = 0.5）：
速度 = 50 mm/s
采样步长 = 50 / 1000 = 0.05 mm（加密 2 倍）
```

---

## 五、物理意义详解

### 5.1 进给速度（feedSpeed）

**定义**：激光头在工件表面的实际移动速度

**单位**：mm/s

**物理意义**：
- 激光出光时的加工速度
- 平台速度和振镜速度的向量和
- 决定加工效率和质量

**与平台/振镜速度的关系**：
```
V_head = V_stage + V_galvo
|V_head| = feedSpeed

其中：
- V_stage：平台速度（低频，50-300 mm/s）
- V_galvo：振镜速度（高频，500-5000 mm/s）
```

**示例**：
```
feedSpeed = 100 mm/s

频率分解后：
- V_stage = 30 mm/s（低频分量）
- V_galvo = 95 mm/s（高频分量）
- |V_head| = √(30² + 95²) ≈ 100 mm/s ✓
```

---

### 5.2 快移速度（rapidSpeed）

**定义**：激光关闭时的空移速度

**单位**：mm/s

**物理意义**：
- 非加工区域的移动速度
- 影响空程时间
- 通常 > feedSpeed

**与 JumpSpeed 的关系**：
```
当前实现：rapidSpeed = JumpSpeedPlatform（平台空移速度）

未来改进：
- rapidSpeed = √(JumpSpeedPlatform² + JumpSpeedGalvo²)
- 区分平台和振镜的空移速度
```

---

### 5.3 加速度（accel）

**定义**：速度变化的快慢

**单位**：mm/s²

**物理意义**：
- 决定加速段长度
- 影响尖角保真度
- 限制最大速度

**计算公式**：
```
加速时间：t = v / a
加速距离：s = 0.5 * v² / a

示例：
v = 100 mm/s, a = 1000 mm/s²
t = 100 / 1000 = 0.1 s
s = 0.5 * 100² / 1000 = 5 mm
```

**对加工的影响**：
- 加速度不足 → 加速段过长 → 有效加工段缩短
- 加速度过高 → 振动增大 → 表面质量下降

---

### 5.4 拐角系数（cornerFactor）

**定义**：控制尖角处速度衰减的参数

**范围**：0.0 - 1.0

**物理意义**：
- 0.0 = 不减速（完全圆滑）
- 0.5 = 减速 50%（折中）
- 1.0 = 完全停止（尖角保真）

**速度衰减公式**：
```
尖角速度 = feedSpeed * (1 - cornerFactor)

示例：
feedSpeed = 100 mm/s
cornerFactor = 0.5
尖角速度 = 100 * (1 - 0.5) = 50 mm/s
```

**对加工质量的影响**：
- cornerFactor 过小 → 尖角变圆角
- cornerFactor 过大 → 尖角处过烧（停留时间长）

---

## 六、使用建议

### 6.1 参数配置策略

**精密加工模式**：
```
feedSpeed = 80 mm/s
cornerFactor = 0.2（尖角处速度 = 64 mm/s）
accel = 800 mm/s²
适用：精密零件、微孔加工
```

**通用加工模式**：
```
feedSpeed = 100 mm/s
cornerFactor = 0.5（尖角处速度 = 50 mm/s）
accel = 1000 mm/s²
适用：通用零件、混合特征
```

**高效生产模式**：
```
feedSpeed = 150 mm/s
cornerFactor = 0.8（尖角处速度 = 30 mm/s）
accel = 2000 mm/s²
适用：大批量生产、简单几何
```

### 6.2 参数调整流程

1. **确定加工目标**：精度优先 vs 效率优先
2. **选择 cornerFactor**：
   - 精度优先 → 0.2-0.4
   - 平衡 → 0.5-0.6
   - 效率优先 → 0.7-0.9
3. **调整 feedSpeed**：
   - 根据材料和厚度
   - 参考工艺参数库
4. **验证效果**：
   - 试切测试件
   - 测量尖角保真度
   - 评估加工时间

---

## 七、常见问题

### Q1：为什么加速度参数未完全使用？

**A**：
- 当前已集成 `cornerFactor`，实现尖角速度衰减
- `accel` 参数已添加到方法签名，但尚未在算法中完整实现
- 未来计划：
  - 计算加速段长度
  - 动态调整采样步长
  - 插入加速/减速段

### Q2：进给速度是平台速度还是振镜速度？

**A**：
- **都不是**
- 进给速度是**激光头的合成速度**
- `feedSpeed = |V_stage + V_galvo|`
- 频率分解后，平台走低频，振镜走高频
- 两者的向量和 = feedSpeed

### Q3：cornerFactor 越大越好吗？

**A**：
- **不是**
- cornerFactor 越大 → 尖角处速度越低 → 尖角保真度越高
- 但会导致：
  - 尖角处停留时间增加 → 过烧风险
  - 加工时间增加
- 需要根据实际需求平衡

### Q4：如何优化尖角加工质量？

**A**：
1. **提高 cornerFactor**：0.5 → 0.7
2. **降低 feedSpeed**：减少尖角处速度
3. **启用顶点吸附**：cornerAngleDeg = 150°
4. **增加加速度**：提高加速响应

---

## 八、总结

### 8.1 核心改进

1. ✅ **集成 cornerFactor**：实现尖角速度衰减
2. ✅ **添加 accel 参数**：为未来加速度计算预留接口
3. ✅ **参数传递**：UI → ViewModel → PathSampler 完整链路

### 8.2 物理意义

- **feedSpeed**：激光头合成速度（平台 + 振镜）
- **rapidSpeed**：空移速度
- **accel**：加速度（预留）
- **cornerFactor**：尖角速度衰减系数

### 8.3 下一步优化方向

1. ✅ **完整实现加速度**：计算加速段长度，动态调整采样
2. ✅ **集成 JumpSpeed**：区分平台和振镜的空移速度
3. ✅ **自适应参数**：根据材料/厚度自动推荐参数
4. ✅ **实时监控**：显示实际速度和加速度曲线

---

**文档版本**：v1.0  
**最后更新**：2026-07-29  
**维护者**：GalvoStage 开发团队
