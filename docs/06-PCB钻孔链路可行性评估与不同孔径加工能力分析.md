# 06 - PCB 钻孔链路可行性评估与不同孔径加工能力分析

> 评估对象：`ImportDrillingFile`（解析）、`PlanDrillingPathAsync`（规划）、`DecomposeDrilling`（分解/仿真）三个环节，
> 核心问题：**方案是否可行？不同孔径的孔能否都被完美加工？**
>
> 本文结论基于当前代码实测梳理，非设计臆测。
>
> 🛠️ **更新（激光钻孔范式确认）**：PCB 钻孔已确认为**激光控制**。`DecomposeDrilling` 已改造为**按孔径环切（trepanning）**——
> 到达孔位后以孔半径沿圆周走 laser on，圈数由停留时间/单圈时间决定，**不同孔径由此产生不同大小的圆周加工轨迹**；
> 孔径未知或过小（半径 < 一个进给步距）时回退为点钻。原致命问题③（运动不体现孔径）已解决，详见 §三.3 / §四。

---

## 一、结论先行

| 维度 | 能否"完美" | 说明 |
|------|-----------|------|
| 孔径数据捕获 | ✅ 可以 | 每孔孔径 `radius×2` 全程保留，统计/分组正确 |
| 激光环切轨迹 | ✅ 已支持 | `DecomposeDrilling` 按孔径画圆环切，不同孔径产生不同轨迹（本次改造） |
| 超大文件全量加工 | ✅ 已支持 | 规划改为全量 Z-order 排序，所有孔进入 `DrillingTrajectory`/G 代码；抽样仅用于仿真预览（本次改造） |
| 超幅面孔 / 单孔跨场 | ✅ 天然支持 | 大圆由平台承载（f=v/2πr 低、a=v²/r 小），连续协同无拼接缝；仅受平台行程/加速度限制（见 §七） |
| 按孔径分组排序 | ⛔ 非必需 | **激光无换刀**，主导成本是孔间跳转空程；纯空间 Z-order 已最优，强行分组反而增大空程（负优化） |
| 差异化工艺参数 | 🟠 待完善 | 环切速度/圈数与孔径解耦程度有限，尚无按孔径的功率/进给表 |

**总体判断**：激光钻孔范式下，孔径已能通过环切轨迹真实体现，超大文件也不再静默丢孔（全量进入加工，抽样只影响预览）。**"按孔径分组排序"是机械钻假设的遗留项，激光范式下非必需甚至负优化，已排除**；唯一剩余关键短板是**按孔径的差异化工艺参数（🟠）**。


---

## 二、端到端链路概览

```
DXF 文件
  │  ImportDrillingFile
  ▼  → DrillingDxfParser.ParseFile（扫描 CIRCLE，radius×2 → Diameter）
DrillingPattern（Holes[]，每孔含 X/Y/Diameter/Layer）
  │  PlanDrillingPathAsync（全量规划，无丢弃）
  ▼  → DrillPlanner.Plan（Z-order 莫顿码 / 网格最近邻，纯空间最短路径）
DrillingTrajectory（HoleMove[]，含 Position/Diameter/DwellTimeMs）
  ├─ DecomposeDrilling → SampleUniform(仅预览子集) → 按孔径环切采样 → FrequencyDecomposer → LinkageSimulator（仿真）
  └─ ExportGCode → GCodeExporter.Export（机械钻备用：按孔径分组换刀 + G81）→ .nc
```

关键观察：**激光主链路 = 全量空间排序 + 按孔径环切**（孔径进入运动指令）；G 代码（机械主轴钻）仅作备用出口，其"按孔径分组换刀"只对机械钻有意义，激光不走此路。

---

## 三、逐环节分析

### 1. 解析 —— `ImportDrillingFile` → `DrillingDxfParser.ParseFile`

**孔径读取本身正确**：读 CIRCLE 组码 10(X)/20(Y)/40(半径)/8(图层)，`radius*2` 存入 `Hole.Diameter`，每孔孔径精确保留。

作为"完美加工"的输入，存在以下会**丢孔或读错孔径**的隐患：

| 隐患 | 说明 |
|------|------|
| 只认 CIRCLE | `ARC`、`INSERT`（块引用钻孔符号）、`POINT`、Excellon/NC 钻带都被忽略 → 整批孔漏读 |
| 单位假定 mm | 忽略 `$INSUNITS`，inch 图纸会整体放大 25.4 倍且不自知 |
| 编码硬解码 UTF8 | ANSI/GBK 中文图层名乱码，影响按图层过滤/分组 |
| `limit = i+24` 行窗口 + 同行/分行启发式 | 带扩展数据的异常 CIRCLE 可能漏读组码 40 → 孔径变 0（未知） |
| 无去重 | 同点重叠圆不会合并 |

### 2. 规划 —— `PlanDrillingPathAsync` → `DrillPlanner.Plan`

✅ **已修复（原致命问题①）：超大文件不再抽样丢孔。**
改造前 `PlanDrillingPathAsync` 对 >10 万孔先用 `SampleHoles`（`i % step == 0`）降到约 10 万孔再规划，抽样后的 `DrillingTrajectory` 正是 G 代码导出源 → 627 万孔只加工约 10 万孔、约 98% 静默丢弃。
现改为**全量规划**：`DrillPlanner.Plan(pattern)` 直接对所有孔做 Z-order（莫顿码）排序，复杂度 O(n log n)，百万级孔亦可承受，全部孔进入 `DrillingTrajectory` → G 代码导出/加工无一丢弃。
原 `SampleHoles` 已重构为通用的 `SampleUniform<T>`，**仅供仿真预览子集使用**（见 §三.3），彻底与加工数据解耦——对齐激光链路"抽样只影响估参/预览，不影响加工指令"的两阶段原则。

⛔ **原"问题②：规划不按孔径分组"——激光范式下已排除，纯空间排序即最优。**
该建议源自**机械钻假设**：机械钻每种孔径要物理换刀（`M6`，秒级），故需分组把同径孔排在一起以摊薄换刀次数。
但激光钻**没有钻头**，孔径由环切半径在软件里决定，换孔径只是改激光参数（μs～ms 指令级），**主导成本变成孔间跳转空程（jump）**：

| | 机械钻 | 激光钻 |
|---|---|---|
| 换孔径代价 | 物理换刀，秒级 | 改参数/环切半径，指令级 |
| 主导成本 | 换刀次数 | 孔间跳转空程 |
| 最优排序 | 先分组、组内再排 | **纯空间最短路径（Z-order / 最近邻）** |

因此对激光而言，强行按孔径分组会把**空间上分散**的同径孔硬凑到一起 → 振镜来回长途奔袭、**空程反而暴增，属负优化**。当前 `Plan` 的纯空间 Z-order 排序即为激光最优，无需改动。
> 仅当不同孔径对应不同激光配方、且配方切换有不可忽略的稳定时间时，才考虑**带权代价排序** `cost = 跳转空程 + λ·参数切换`（λ 随切换成本调节），而非无脑聚类；本项目参数切换极廉价，λ→0 退化为纯空间排序。

### 3. 分解/仿真 —— `DecomposeDrilling`

✅ **已修复（原致命问题③）：运动链路现按孔径环切。**
改造前采样只用 `m.Position` 与 `m.DwellTimeMs`，`m.Diameter` 从未被使用，每孔退化为"移动到点 → 原地零半径停留"，Ø0.2 与 Ø3.0 物理上完全一样。
现已改为**激光环切（trepanning）**：

```
r = Diameter/2
若 Diameter>0 且 r ≥ 一个进给步距(feedStep = FeedSpeed·dt)：
    空移(laser off)到进刀点 (cx+r, cy)
    沿半径 r 圆周走 laser on，每圈点数 = ⌈2πr / feedStep⌉（下限 8）
    圈数 loops = round(DwellTimeMs / 单圈时间)   // 体现孔径 + 加工量
否则（孔径未知或过小）：
    回退为原地点钻停留
```

- **不同孔径产生不同大小的圆周轨迹**，Ø 越大圆越大、单圈点数越多 → 真实体现孔径；
- 停留时间换算为**环切圈数**，孔径 + 加工量同时反映在轨迹里；
- 圆周仍是等时（等弧长）采样，直接进入既有频率分解链路，无需额外改动；
- `PlanInfo` 增加"环切 N 孔 / 点钻 N 孔"统计，便于核对。

🟡 其余：`MaxSimHoles = 2000` 抽样仅用于**仿真预览**（可接受，但不代表全量运动学）；环切速度/圈数尚未按孔径做差异化功率与进给（见 §四）。


---

## 四、不同孔径能否"完美加工"——三条链路拆解

- **数据层**：✅ 区分。孔径精确解析、按 3 位小数（μm 级）归类统计，无信息丢失。
- **激光环切层**（`DecomposeDrilling`）：✅ 体现孔径。按孔半径生成圆周轨迹，不同孔径 → 不同大小的圆、不同单圈点数，停留时间转为环切圈数；已并入频率分解链路。
  - 🟠 尚待完善：环切**进给速度、圈数、激光功率未按孔径分档**——大孔可能需要多层螺旋/变功率，小孔需限制圈数防过烧。当前 loops 仅由 DwellTimeMs 推导，功率无区分。
- **机械 G 代码层**（`GCodeExporter.Export`，如仍需机械钻备用）：⚠️ 能换刀，不能"完美"。
  - ✅ 按孔径分组 → 每组 `Tn M6` 换刀 + `G81` 循环（分组仅对机械钻有意义，激光不走此路，见 §三.2）；
  - ❌ **全孔径共用同一 `Options`**（深度/进给/转速/退刀相同）→ 小孔易断刀、大孔易烧焦。

---

## 五、风险清单（按严重度）

| 级别 | 问题 | 位置 |
|------|------|------|
| ✅ | ~~>10 万孔按索引抽样静默丢孔~~ 已改为全量规划，抽样仅用于仿真预览 | `PlanDrillingPathAsync` / `SampleUniform` |
| ✅ | ~~分解/仿真运动不使用孔径~~ 已改为按孔径环切 | `MainViewModel.DecomposeDrilling` |
| 🟠 | 环切工艺（进给/圈数/功率）未按孔径分档 | `MainViewModel.DecomposeDrilling` |
| ⛔ | ~~规划不按孔径分组~~ 激光范式下非必需/负优化，纯空间排序即最优（见 §三.2） | `DrillPlanner.Plan` |
| 🟠 | 机械 G 代码各孔径共用同一工艺参数（如启用机械钻备用） | `GCodeExporter.Options` |
| 🟡 | 解析仅支持 CIRCLE；单位/编码/行窗口假定 | `DrillingDxfParser` |
| 🟡 | `DwellTimeMs`、`TotalDurationMs`（50ms/孔）不随孔径/实际调整 | `Plan` / `DecomposeDrilling` |

---

## 六、改进建议（优先级排序）

1. ~~**先修丢孔**~~ ✅ **已完成**：`PlanDrillingPathAsync` 改为全量 Z-order 规划，所有孔进入 `DrillingTrajectory`/G 代码导出；`SampleUniform` 仅用于仿真预览子集。"规划/导出"与"仿真"已解耦，对齐激光两阶段方案。
2. ~~**明确工艺范式**~~ ✅ **已确认为激光钻**：按孔径生成 trepanning 圆环轨迹，孔径已进入运动指令。
3. ~~**规划按孔径分组**~~ ⛔ **激光范式下已排除**：激光无换刀，主导成本是孔间跳转空程，纯空间 Z-order 排序即最优，强行分组反而增大空程（负优化）。仅当激光配方切换成本显著时才用带权代价排序（见 §三.2），当前无需。
4. **孔径差异化工艺（🟠 当前最高）**：环切进给/圈数/激光功率升级为"按孔径的参数表"，不同孔径各用其参数（大孔多层螺旋、小孔限圈防过烧）。
5. **解析健壮性**：支持 ARC/INSERT/POINT、`$INSUNITS` 单位换算、编码探测。

---

## 七、边界工况分析：孔径大于振镜幅面 / 单孔跨场拼接

### 7.1 机理回顾：为什么"大圆"天然落在平台

环切轨迹是绕孔心的圆 `x(t)=cx+r·cosωt, y(t)=cy+r·sinωt`，角频率 `ω=v/r`（v=FeedSpeed），故**环切频率 `f = v/(2πr)`**。频率分解按截止频率 fc 拆分（`FrequencyDecomposer`）：
- 低频（<fc）→ **平台**（大行程、无视场限制）；
- 高频残差（`galvo = 原信号 − 平台低通`）→ **振镜**（受视场约束，`MaxGalvoDeviation ≤ GalvoFov`）。

`DecomposeAuto` 二分搜索"能让振镜残差 ≤ FoV·0.8 的**最低**截止频率"。三个关键量纲：

| 量 | 公式 | 随孔径 r |
|----|------|----------|
| 环切频率 | f = v/(2πr) | **r 越大 f 越低** |
| 平台切向速度 | = v（恒等于进给速度） | 与 r 无关 |
| 平台向心加速度 | a = v²/r | **r 越大 a 越小** |

结论：**大孔 = 低频 + 低加速度**，正是平台舒适区。只要 fc 取在 f 之上，平台就能完整跟踪整个圆，振镜残差趋近 0——**"超幅面的圆由平台画出来"**。

### 7.2 情况一：孔径大于振镜幅面（2r > 幅面）

**✅ 架构天然支持，这正是振镜+平台协同的设计目的。**
2r 超过幅面时 r 必然较大 → f=v/(2πr) 很低 → 平台低通完整承载大圆，振镜仅做微量高频修正，不存在"振镜画不下"。

真正的约束不是视场，而是平台能力：

| 约束 | 触发条件 | 后果 / 对策 |
|------|----------|-------------|
| 平台行程 | r 超出 XY 平台可达范围 | 物理不可加工，任何方案皆无解 |
| 平台加速度 | a=v²/r 超平台能力（多为**小孔高进给**） | 降低该孔进给 v，a 立即随平方下降 |
| 自动截止上限 | f > fcHigh(=min(60Hz, fs·0.45)) 平台跟不上 | 仅"半径大+进给极高"的极端组合出现；降进给即可 |

数值示例（幅面≈12mm 半场、v=500mm/s、fs=1kHz）：
- **Ø30mm 孔**（r=15>12）：f=5.3Hz，a≈16,700mm/s²（~1.7g）。fc 自动升至 >5.3Hz，平台画整圆、振镜≈0 → **可行**；
- **Ø200mm 孔**（r=100）：f=0.8Hz，a=2,500mm/s²（0.25g），只要平台行程≥100mm → **可行**；
- **反例**：r=15mm 且 v=5000mm/s → f=53Hz、a≈170g，平台无法跟随 → 需大幅降进给。**瓶颈是平台动力学，不是振镜视场。**

### 7.3 情况二：单孔需要"两个场拼接"

**⛔ 本架构不存在"拼接"概念，因而没有拼接缝问题。**
传统纯振镜设备视场固定（如 100×100mm），超场图形要"平台步进→静止→振镜重扫→拼接"，接缝处有套准误差。
本项目用**连续频率协同**：平台始终连续运动承载低频大行程，振镜叠加高频，**等效视场 = 平台行程**。因此：
- 单个大孔是**一条连续的圆**（平台画大圆 + 振镜微修），从不被切成两个场 → **无拼接缝、无套准误差**；
- 代价是平台全程动态跟随（而非静止分场），其跟踪误差由平台带宽/阻尼决定，已在 `LinkageSimulator` 中建模。

唯一"拼接也无解"的情形：**孔尺寸超过平台物理行程**——机床行程上限，拼接救不了（造不出比机床行程还大的孔）。

### 7.4 逐孔动力学预检（已实现）

- 🟡 **全局单一截止频率**：所有孔共用一个 fc；含超大孔时 fc 被抬到其环切频率之上（幅度不大，属隐式耦合）。
- 🟡 **无按孔自适应进给**：不会对"半径大+加速度超限"的孔自动降速，需人工调 FeedSpeed（预检已给出建议值）。

✅ **已落地**：算法下沉至 Core 层 `DrillDynamicsPrecheck.Evaluate`（纯函数，可单测），返回结构化结果 `DrillDynamicsReport`（携带 `MaxRingcutFrequencyHz`、`MaxCentripetalAccel`、`FrequencyCapHz` 及按孔径聚合的 `Offenders` 明细）。`MainViewModel.PrecheckDrillDynamics` 在 `DecomposeDrilling` 中对**全量孔**调用它，将报告缓存到 `LastDrillDynamics` 供 UI/其它消费者读取数值，并格式化写入 `PlanInfo`，弥补"`MaxGalvoDeviation` 只有全局最大值、不知是哪个孔越界"的缺口。判据与逻辑：

```
对每个孔（半径 r = Diameter/2）：
    若 r ≤ GalvoFov        → 视场内，振镜独立覆盖，恒可行（跳过）
    否则 f = v/(2πr)        → 环切频率
         若 f > fCap        → 平台带宽跟不上，高频残差落到视场外 → 标记越界
fCap = min(StageBandwidth·0.8, fs·0.45)   // 平台可稳定跟随的环切频率上限
```

- 按**孔径聚合**越界孔（避免逐孔刷屏），每种孔径报告 `f`、向心加速度 `a=v²/r` 与**建议进给** `v_suggest = 2π·r·fCap`（使 f 落回 fCap 的上限进给）；
- 全部可行时显示"逐孔预检：✅ 全部孔径在当前进给下可行"；
- 只检查平台动力学，**不检查视场**（大孔由平台承载，视场对其非约束，见 §7.1～7.2）；
- 平台**物理行程**无对应建模参数，未纳入硬判据（超行程属机床上限，见 §7.3）。

---

## 八、相关源码位置

| 环节 | 文件 | 关键方法 |
|------|------|----------|
| 导入 | `src/GalvoStage.App/ViewModels/MainViewModel.cs` | `ImportDrillingFile` |
| 解析 | `src/GalvoStage.Core/Dxf/DrillingDxfParser.cs` | `ParseFile` |
| 数据模型 | `src/GalvoStage.Core/Geometry/Drilling/DrillingPattern.cs` | `Hole` / `RecomputeBounds` |
| 规划 | `src/GalvoStage.App/ViewModels/MainViewModel.cs` | `PlanDrillingPathAsync`（全量规划）/ `SampleUniform`（仅预览抽样） |
| 排序 | `src/GalvoStage.Core/Drilling/DrillPlanner.cs` | `Plan` / `OrderByZonal` / `OrderByNearestGrid` |
| 分解仿真 | `src/GalvoStage.App/ViewModels/MainViewModel.cs` | `DecomposeDrilling` |
| 逐孔预检 | `src/GalvoStage.Core/Drilling/DrillDynamicsPrecheck.cs` | `Evaluate` → `DrillDynamicsReport`（f/a 统计 + 越界明细 + 建议进给） |
| 预检接入 | `src/GalvoStage.App/ViewModels/MainViewModel.cs` | `PrecheckDrillDynamics`（调用 Core 预检、缓存 `LastDrillDynamics`、格式化） |
| 频率分解 | `src/GalvoStage.Core/PathPlanning/FrequencyDecomposer.cs` | `Decompose` / `DecomposeAuto`（视场约束、截止频率二分） |
| 联动仿真 | `src/GalvoStage.Core/Simulation/LinkageSimulator.cs` | 平台带宽/阻尼跟踪误差建模 |
| 导出 | `src/GalvoStage.Core/Drilling/GCodeExporter.cs` | `Export` |

---

## 附：一句话结论

> PCB 钻孔已确认为**激光控制**：`DecomposeDrilling` 现按孔径做**环切（trepanning）**，不同孔径生成不同大小的圆周加工轨迹，孔径已真实进入运动指令并接入频率分解链路。规划阶段已改为**全量 Z-order 排序**，所有孔进入 `DrillingTrajectory` 与 G 代码导出，超大文件不再静默丢孔（`SampleUniform` 抽样仅用于仿真预览，不影响加工）。孔径数据全程无损。**"按孔径分组排序"是机械钻遗留假设，激光范式下非必需甚至负优化（纯空间排序即最优），已排除。** **孔径大于振镜幅面、单孔跨场等边界工况天然由"平台画大圆 + 振镜微修"的连续协同承载（大孔 = 低频 + 低加速度），无拼接缝，仅受平台行程/加速度限制而非视场（见 §七）。** 唯一剩余关键短板：**环切工艺（进给/圈数/功率）尚未按孔径分档（🟠）**。
