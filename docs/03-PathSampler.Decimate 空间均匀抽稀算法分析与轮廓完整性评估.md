# PathSampler.Decimate 空间均匀抽稀算法分析与轮廓完整性评估

> 面向对象：需要理解「超大规模轮廓集合抽稀」原理，并评估其对加工结果影响的开发者 / 工艺人员。
>
> 相关源码：[`PathSampler.Decimate`](../src/GalvoStage.Core/PathPlanning/PathSampler.cs)、调用点 [`MainViewModel.Decompose`](../src/GalvoStage.App/ViewModels/MainViewModel.cs)

---

## 一、方法定位（先说结论）

`Decimate` 是一个 **「轮廓级」抽稀（下采样）** 算法：

- 它的操作粒度是 **整条折线（PathPolyline）**：被选中的轮廓 **原封不动、逐点完整保留**；未被选中的轮廓 **整条丢弃**。
- 它 **不会** 修改任何一条被保留轮廓的形状（不做点抽稀、不做拟合、不做简化）。
- **重要（2026-07 架构订正）**：抽稀 **只用于「参数估计」**——即在代表子集上快速求出频率分解的截止频率；
  最终产出运动指令的 `Decompose()` 已改为 **两阶段**：子集估参 + **全量单次分解**，因此 **最终双轴联动指令覆盖全部轮廓，不丢任何一条**。

因此对用户提出的两个问题，先给出直接答案：

| 问题 | 结论 |
|------|------|
| 采用空间均匀抽稀后，实际加工轮廓与导入 DXF 是否有出入？ | **无。** `Decompose()` 采用「子集估参 + 全量分解」两阶段：抽稀子集只用于求截止频率，最终 `FrequencyDecomposer.Decompose` 吃 **全量轮廓**，运动指令与 DXF 一一对应。 |
| 是否存在漏掉部分轮廓的可能？ | **不会。** 抽稀子集仅参与「估截止频率」，不参与最终指令生成；全量分解保证每条轮廓都被加工。（子集内部虽丢弃 `n − maxCount` 条，但这只影响估参精度，不影响加工完整性。） |

下文详述算法与量化评估。

---

## 二、算法详解

### 2.1 方法签名与触发条件

```csharp
public static List<PathPolyline> Decimate(IReadOnlyList<PathPolyline> polylines, int maxCount)
```

- 输入：全量轮廓集合 `polylines`、目标上限 `maxCount`。
- 输出：数量 ≤ `maxCount` 的代表性子集（保留的轮廓为原对象引用，未做拷贝）。
- 短路：`n ≤ maxCount` 时原样返回，零开销。
- 调用现场（唯一）：

```csharp
if (source.Count > PathSampler.MaxSampleContours)          // MaxSampleContours = 20_000
{
    source = PathSampler.Decimate(source, PathSampler.MaxSampleContours);
    decimateNote = $"轮廓抽稀：{Polylines.Count:N0} → {source.Count:N0}（仿真代表子集）";
}
var traj = PathSampler.Sample(source, ...);                // 仅供仿真/频率分解
```

### 2.2 核心思想

目标不是「随机抽一批」，而是 **在版图空间上均匀铺开**——保证抽出来的子集在整块板/幅面里到处都有代表，几何分布特征（密集区、稀疏区、整体轮廓）与全量尽量一致，从而让仿真估算的截止频率、平台速度/加速度、振镜偏摆等指标具有代表性。

实现分四步：**代表点求包围盒 → 建方形网格 → 计数排序分桶 → 逐桶轮流取样**。

### 2.3 步骤拆解

**① 以「首点」为代表求全局包围盒**

```csharp
Vec2 p = pts[0];   // 每条轮廓只用第 0 个点代表其位置
// 更新 minX/minY/maxX/maxY
```

> ⚠️ 关键近似：分桶只看每条轮廓的 **首点**，不看轮廓的实际跨度。含义见 §3.3。

**② 建 √maxCount × √maxCount 方形网格**

```csharp
int dim = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(maxCount)));   // maxCount=20000 → dim=142
double cw = Math.Max((maxX - minX) / dim, 1e-9);                 // 单元宽
double ch = Math.Max((maxY - minY) / dim, 1e-9);                 // 单元高
int bucketCount = dim * dim;                                     // ≈ maxCount 个桶
```

桶数 ≈ 目标数量，意味着「平均每个桶最终贡献约 1 条轮廓」。

**③ 计数排序分桶（避免海量小对象分配）**

用一维 `counts` 前缀和 + `grouped` 索引数组完成分桶，是标准的 counting sort：

```csharp
// 落桶
int cx = Math.Clamp((int)((pts[0].X - minX) / cw), 0, dim - 1);
int cy = Math.Clamp((int)((pts[0].Y - minY) / ch), 0, dim - 1);
cellOf[i] = cy * dim + cx;
counts[cell + 1]++;
// 前缀和 → 每桶起始下标；再把原始索引按桶写入 grouped[]
```

`grouped` 内同一桶的轮廓 **保持原始输入顺序**（counting sort 对同桶元素稳定）。

**④ 逐桶轮流取样（round-robin），直到满额**

```csharp
for (int pass = 0; result.Count < maxCount; pass++)
{
    bool any = false;
    for (int b = 0; b < bucketCount && result.Count < maxCount; b++)
    {
        int idx = counts[b] + pass;      // 第 pass 轮取该桶的第 pass 条
        if (idx >= cursor[b]) continue;  // 该桶已取空
        result.Add(polylines[grouped[idx]]);
        any = true;
    }
    if (!any) break;                     // 全部桶取空
}
```

- **pass 0**：每个非空桶各取「第 1 条」→ 保证空间全覆盖。
- **pass 1、2…**：对仍有剩余的密集桶继续取第 2、3…条，直到攒够 `maxCount`。
- 稀疏桶（1 条轮廓）只在 pass 0 贡献；密集桶（几十条）会在多轮里持续贡献。

### 2.4 复杂度

| 维度 | 量级 | 说明 |
|------|------|------|
| 时间 | O(n) | 四步均为线性扫描；round-robin 总取样数 = maxCount ≤ n |
| 空间 | O(n + maxCount) | `cellOf/grouped` 各 n，`counts/cursor` 各 ≈ maxCount |
| 分配 | 常数级大数组 | 无 `List<List<>>` 小对象海量分配 |

对 60 万、800 万级 DXF 轮廓，抽稀本身耗时可忽略（相对解析与采样）。

---

## 三、轮廓完整性评估（核心）

### 3.1 「保留」的轮廓：与 DXF 逐点一致

被选中的轮廓是 **原始 `PathPolyline` 引用**，`Points` 未做任何删改。因此：

- 被保留轮廓的形状、点数、闭合性、图层 **与导入 DXF 完全一致**，无任何几何失真。
- 抽稀 **不是** 点级简化，不存在「圆变多边形」「拐角被抹平」这类误差。

### 3.2 「丢弃」的轮廓：确实会漏，但有均匀性保证

抽稀的本质就是丢弃 `n − maxCount` 条轮廓，这是 **预期行为**。需要评估的是「漏得是否均匀、会不会整片丢失」：

- **不会整片区域消失**：桶数 ≈ maxCount，pass 0 会让 **每个非空网格单元至少贡献 1 条轮廓**。也就是说，只要某个空间区域原本有轮廓，抽稀后该区域至少还留有一条代表，不会出现「整块角落全被删掉」。
- **密集区按比例减配**：孔/轮廓越密的桶，被丢弃的比例越高（因为它超出了「每桶均摊份额」）。丢弃的是该桶中 **原始输入顺序靠后** 的轮廓（稳定分桶决定）。
- **稀疏区几乎全保留**：只有 1~2 条轮廓的桶，通常全部保留。

### 3.3 已知偏差来源（首点代表法）

分桶只用每条轮廓的 **首点** 定位，带来两点近似：

1. **大跨度轮廓的定位偏差**：一条横跨整块板的长折线，仅按其首点归入某个桶。它的空间「影响范围」被低估，可能与真实几何中心不符。对「均匀覆盖」目标影响有限（长轮廓通常本就该保留），但统计上存在偏移。
2. **空点轮廓**：`pts.Count == 0` 的轮廓 `cellOf = -1`，被直接排除（本就无几何意义）。

这些偏差 **只影响「代表子集的统计代表性」**，不影响任何被保留轮廓的几何精度。

### 3.4 对实际加工的影响：无（两阶段分解）

激光模式产出运动指令的唯一管线就是 `MainViewModel.Decompose()`（没有独立的激光 G 代码导出，`GCodeExporter` 仅服务钻孔模式）。为兼顾交互性能与加工完整性，它已重构为 **两阶段**：

```
阶段① 估参（可抽稀）：[>20000 时 Decimate] → Sample(子集) → DecomposeAuto → 仅取截止频率 fc
阶段② 指令（必全量）：Sample(全量 Polylines) → Decompose(全量, fc) → Plan → 双轴联动
```

- 昂贵的 `DecomposeAuto`（约 18 次 filtfilt 二分迭代）只在代表子集上跑，保证交互性能。
- 最终 `Decompose(全量, fc)` 是 **单次 filtfilt**（O(n) 一遍），输入为 **全部轮廓**，产出的 `Plan`（StageX/Y、GalvoX/Y）覆盖每一条轮廓。
- 截止频率是全局频谱参数，用代表子集估得足够准；分解后界面报告的是 **全量集的真实振镜偏摆 / 峰值速度 / 加速度**（若超视场会提示“× 超出视场”）。

> 结论：**抽稀仅影响“估截止频率”，不参与最终指令生成；全量分解保证加工覆盖全部轮廓，与 DXF 无出入、不漏轮廓。**

---

## 四、风险场景与建议

| 场景 | 是否受影响 | 说明 |
|------|-----------|------|
| 正常加工（≤20000 轮廓） | 否 | 直接全量 DecomposeAuto，无抽稀 |
| 大规模加工（>20000 轮廓） | 否 | 子集仅估截止频率，最终全量单次分解，指令覆盖全部轮廓 |
| 估参精度 | 轻微 | 截止频率在代表子集上估得，可能与全量最优值有微小偏差；界面按全量真实指标复核，超视场会提示 |
| 未来若要「加工级 LOD」减量 | —— | **不要用 Decimate 丢轮廓**，改用轮廓级点简化（见下） |

### 4.1 若确需在加工侧降数据量，应换算法

`Decimate` 丢整条轮廓，不适合加工。若将来需要减少加工数据量而 **保留所有轮廓**，应采用 **点级简化**（每条轮廓内部抽稀），例如 Douglas–Peucker：

| 方案 | 粒度 | 轮廓数 | 单轮廓形状 | 适用 |
|------|------|--------|-----------|------|
| `Decimate`（现状） | 整条取舍 | 减少 | 完全保真 | 仿真/预览统计 |
| Douglas–Peucker | 轮廓内点 | 不变 | 受控误差(ε) | 加工数据瘦身 |

Douglas–Peucker 保证「不漏任何一条轮廓」，且形状误差可由容差 ε 严格界定（如 ε = 0.5×光斑直径），是加工侧减量的正确方向。

---

## 五、验证建议

1. **完整性单测**：构造 n > maxCount 的集合，断言 Decimate `result.Count == maxCount`、结果均为原始引用、每个非空桶至少 1 条。
2. **加工完整性回归**：对 >20000 轮廓集合调用 `Decompose()`，断言 `Plan.Count` 对应全量采样（而非子集），PlanInfo 显示「加工轮廓：N 条（全量，无丢弃）」。
3. **估参一致性**：对比「子集估参 + 全量分解」与「全量 DecomposeAuto」两者的截止频率与振镜偏摆，确认偏差在可接受范围。

---

## 附：关键结论一句话

> `Decimate` 是「空间均匀、整条轮廓取舍」的下采样，**仅用于在代表子集上快速估计截止频率**；`Decompose()` 两阶段（子集估参 + 全量单次分解）保证 **最终双轴联动指令覆盖全部轮廓，与导入 DXF 无出入、不漏任何一条轮廓**。
