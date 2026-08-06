# API 参考文档

> `GalvoStage.Core` 核心算法库公开 API 参考。所有类型无 UI 依赖，可在任意 .NET 6 项目中引用。
> 命名空间前缀：`GalvoStage.Core.*`。长度单位统一为 **mm**，时间单位为 **s**，频率为 **Hz**。

## 目录

- [Geometry — 几何基元](#geometry--几何基元)
  - [Vec2](#vec2)
  - [PathPolyline](#pathpolyline)
- [Dxf — DXF 解析](#dxf--dxf-解析)
  - [DxfParser](#dxfparser)
- [PathPlanning — 路径规划](#pathplanning--路径规划)
  - [SampledTrajectory](#sampledtrajectory)
  - [PathSampler](#pathsampler)
  - [DecomposeResult](#decomposeresult)
  - [FrequencyDecomposer](#frequencydecomposer)
- [Drilling — 钻孔路径规划](#drilling--钻孔路径规划)
  - [DrillingPattern](#drillingpattern)
  - [TrepanParams](#trepanparams)
  - [DrillingStrategy](#drillingstrategy)
  - [DrillPlanner](#drillplanner)
- [Simulation — 动力学仿真](#simulation--动力学仿真)
  - [StageAxisModel](#stageaxismodel)
  - [GalvoAxisModel](#galvoaxismodel)
  - [LinkageSimulator](#linkagesimulator)
- [完整调用示例](#完整调用示例)

---

## Geometry — 几何基元

命名空间：`GalvoStage.Core.Geometry`

### Vec2

二维矢量（`readonly struct`，单位 mm）。

| 成员 | 类型 | 说明 |
|------|------|------|
| `X`, `Y` | `double` | 分量（只读字段） |
| `Vec2(double x, double y)` | 构造 | 创建矢量 |
| `Zero` | `static Vec2` | 零矢量 `(0,0)` |
| `Length` | `double` | 模长 $\sqrt{x^2+y^2}$ |
| `DistanceTo(Vec2 other)` | `double` | 到另一点的欧氏距离 |

**运算符**：`+`、`-`、`*`（矢量×标量，两侧均可）、`/`（矢量÷标量）。实现 `IEquatable<Vec2>`。

```csharp
var a = new Vec2(3, 4);
double len = a.Length;            // 5.0
var b = a * 2 + Vec2.Zero;        // (6, 8)
double d = a.DistanceTo(b);       // 5.0
```

### PathPolyline

连续折线——所有 DXF 实体细分后的统一表示。

| 成员 | 类型 | 说明 |
|------|------|------|
| `Points` | `List<Vec2>` | 顶点序列 |
| `Closed` | `bool` | 是否闭合（首尾相连） |
| `Layer` | `string` | 所属图层名（默认 `"0"`） |
| `FromCircle` | `bool` | 是否由 CIRCLE 实体细分而来（混合解析中 CIRCLE 同时写入折线与钻孔两份数据）。双模式加工时圆孔由钻孔链路环切处理，此标记用于将其从折线链路排除，避免圆被加工两次。默认 `false` |
| `Length` | `double` | 折线总长（闭合时含收尾段） |

---

## Dxf — DXF 解析

命名空间：`GalvoStage.Core.Dxf`

### DxfParser

静态类。轻量 ASCII DXF 解析器，读取 `ENTITIES` 段并将实体统一细分为 `PathPolyline`。

| 方法 | 签名 | 说明 |
|------|------|------|
| `ParseFile` | `static List<PathPolyline> ParseFile(string path)` | 从文件路径解析（折线模式） |
| `Parse` | `static List<PathPolyline> Parse(Stream stream)` | 从字节流解析（折线模式） |
| `ParseFileMixed` | `static MixedParseResult ParseFileMixed(string path)` | **双模式分离解析**：一次遍历同时提取折线特征与钻孔特征 |

| 常量 | 值 | 说明 |
|------|-----|------|
| `ChordTolerance` | `0.01` | 圆弧细分弦高误差（mm） |

**支持实体**：`LINE`、`CIRCLE`、`ARC`、`LWPOLYLINE`（含凸度 bulge）、`POLYLINE`/`VERTEX`、`ELLIPSE`、`SPLINE`（NURBS/拟合点）、`INSERT`（块引用展开，`BLOCKS` 段定义 + 平移/旋转/缩放实例化）。

采用**单遍字节流式解析**：直接在字节缓冲区上解析组码与数值，不为数值分配字符串、不一次性把全部组码读入内存，可在数秒内处理数百 MB 大文件。

```csharp
List<PathPolyline> shapes = DxfParser.ParseFile(@"C:\part.dxf");
Console.WriteLine($"轮廓数={shapes.Count}, 总长={shapes.Sum(p => p.Length):F1} mm");
```

#### MixedParseResult

双模式分离解析结果：折线特征（轮廓）与钻孔特征（小圆）分开存放。CIRCLE 实体**同时**写入两份数据——钻孔链路（作为孔）与折线链路（作为闭合圆轮廓，且 `FromCircle=true`）——其他实体只写入折线数据。

| 成员 | 类型 | 说明 |
|------|------|------|
| `Polylines` | `List<PathPolyline>` | 折线特征（LWPOLYLINE / LINE / ARC / ELLIPSE / SPLINE / POLYLINE / INSERT 展开 / CIRCLE 轮廓） |
| `DrillingHoles` | `DrillingPattern` | 钻孔特征（CIRCLE 作为孔） |
| `CircleCount` | `int` | CIRCLE 总数 |

```csharp
var mixed = DxfParser.ParseFileMixed(@"C:\board.dxf");
Console.WriteLine($"轮廓 {mixed.Polylines.Count} 条，孔 {mixed.DrillingHoles.Holes.Count} 个");
```

> **注**：仅支持 ASCII DXF；不支持二进制 DXF。未识别实体会被忽略。

---

## PathPlanning — 路径规划

命名空间：`GalvoStage.Core.PathPlanning`

### SampledTrajectory

等时间间隔采样后的加工轨迹（联动控制的指令基准）。所有数组等长。

| 成员 | 类型 | 说明 |
|------|------|------|
| `SampleRate` | `double` | 采样/控制频率（Hz） |
| `Dt` | `double` | 采样周期 = `1/SampleRate` |
| `X`, `Y` | `double[]` | 各采样点坐标 |
| `LaserOn` | `bool[]` | 各点激光是否出光（区分轮廓/空程） |
| `Count` | `int` | 采样点数 |
| `Duration` | `double` | 总时长 = `Count × Dt`（s） |

### PathSampler

静态类。将折线集合转换为等时采样轨迹。

```csharp
static SampledTrajectory Sample(
    IReadOnlyList<PathPolyline> polylines,
    double feedSpeed,            // 进给速度 mm/s（轮廓，激光开）
    double jumpSpeedPlatform,    // 平台空移速度 mm/s
    double jumpSpeedGalvo,       // 振镜空移速度 mm/s
    double sampleRate,           // 采样率 Hz
    double cornerAngleDeg = 150, // 尖角保真阈值（内角 < 此值的顶点强制吸附）；≥180 关闭
    double accelPlatform = 1000, // 平台加速度 mm/s²
    double accelGalvo = 5000,    // 振镜加速度 mm/s²
    double cornerFactor = 0.5,   // 拐角系数 0-1：尖角处速度衰减 0=不减 1=完全停
    double decelPlatform = 0)    // 平台减速度 mm/s²（0=同 accelPlatform）
```

- 空移合成速度 `rapidSpeed = min(√(jumpSpeedPlatform² + jumpSpeedGalvo²), 1000)` mm/s（平台与振镜向量和，并限速 1000）。
- **内部处理**：① 贪心最近邻排序（可整体反向折线；>5000 条自动切网格加速）减少空程；② 空程/轮廓按各自速度等距插补；③ 跨段相位连续；④ 尖角保真（顶点吸附）。

```csharp
var traj = PathSampler.Sample(shapes, feedSpeed: 80,
    jumpSpeedPlatform: 500, jumpSpeedGalvo: 2000, sampleRate: 1000);
```

| 其他方法 | 说明 |
|----------|------|
| `Decimate(polylines, maxCount)` | 空间均匀抽稀：按 √maxCount×√maxCount 网格分桶后逐桶轮流取样，得到覆盖全版图的代表性子集（≤maxCount 时原样返回） |

| 常量 | 值 | 说明 |
|------|-----|------|
| `MaxSampleContours` | `20_000` | 采样/仿真的轮廓数上限，超过时应先 `Decimate` |

### DecomposeResult

频率分解结果：平台低频分量 + 振镜高频分量 + 可行性指标。

| 成员 | 类型 | 说明 |
|------|------|------|
| `Raw` | `SampledTrajectory` | 原始采样轨迹（理想目标） |
| `StageX`, `StageY` | `double[]` | 平台指令（低频分量） |
| `GalvoX`, `GalvoY` | `double[]` | 振镜指令（高频残差，相对平台） |
| `CutoffHz` | `double` | 实际使用的截止频率 |
| `MaxGalvoDeviation` | `double` | 振镜最大偏摆（mm）——需 ≤ 视场 |
| `StageMaxVelocity` | `double` | 平台峰值速度（mm/s） |
| `StageMaxAcceleration` | `double` | 平台峰值加速度（mm/s²） |
| `Count` | `int` | 采样点数 |

### FrequencyDecomposer

静态类。基于零相位二阶 Butterworth 低通滤波的频率分解。

| 方法 | 说明 |
|------|------|
| `Decompose(traj, cutoffHz, galvoFov)` | 以**指定截止频率**分解 |
| `DecomposeAuto(traj, galvoFov, margin=0.8, fcLow=0.2, fcHigh=60)` | **自动二分搜索**满足视场约束的最低截止频率 |

```csharp
// 指定截止频率
static DecomposeResult Decompose(
    SampledTrajectory traj, double cutoffHz, double galvoFov);

// 自动搜索（推荐）
static DecomposeResult DecomposeAuto(
    SampledTrajectory traj, double galvoFov,
    double margin = 0.8,   // 视场安全裕度
    double fcLow = 0.2,    // 搜索下限 Hz
    double fcHigh = 60);   // 搜索上限 Hz
```

- `cutoffHz` 会被限幅至 `[0.1, SampleRate × 0.45]`。
- `DecomposeAuto` 目标：`MaxGalvoDeviation ≤ galvoFov × margin`，在此前提下最小化平台加速度。若上限仍超视场，返回上限最优解（提示该图形无法在当前视场下完全联动）。

```csharp
var plan = FrequencyDecomposer.DecomposeAuto(traj, galvoFov: 5);
if (plan.MaxGalvoDeviation > 5)
    Console.WriteLine("警告：振镜偏摆超出视场，需增大视场或降低进给速度");
```

---

## Drilling — 钻孔路径规划

命名空间：`GalvoStage.Core.Drilling`（钻孔点集位于 `GalvoStage.Core.Geometry.Drilling`）。

### DrillingPattern

PCB 钻孔点集数据模型（与 `PathPolyline` 并列的独立加工模式，只处理离散孔位）。

| 成员 | 类型 | 说明 |
|------|------|------|
| `Holes` | `List<Hole>` | 孔位列表 |
| `Bounds` | `(MinX,MinY,MaxX,MaxY)?` | 全局包围盒（缓存） |
| `LayerCounts` | `Dictionary<string,int>` | 按图层分组计数 |
| `DiameterCounts` | `Dictionary<double,int>` | 按孔径分组计数（保留 3 位小数归类） |
| `RecomputeBounds()` | `void` | 重算包围盒、图层与孔径统计 |

**Hole 结构**：`X`、`Y`、`Diameter`（0=未知）、`Layer`、`OriginalIndex`、`ProcessParams`；`RecomputeProcessParams()` 按孔径自动选择工艺参数分档。

### TrepanParams

激光钻孔环切工艺参数（按孔径分档配置，`init` 只读）。

| 成员 | 类型 | 说明 |
|------|------|------|
| `Power` | `double` | 激光功率（W） |
| `OffsetRings` | `int` | 补偿圈数（扩孔层数） |
| `FeedRate` | `double` | 进给速度（mm/s） |
| `HoldTime` | `double` | 持留时间（ms） |
| `CoolDownInterval` | `double` | 冷却间隔（ms） |
| `DutyCycle` | `double` | 脉冲占空比（0-1） |

**分档预设**（`CreateForDiameter(d)` 自动选择）：

| 预设 | 孔径 d | Power | Rings | Feed | Hold | Cool | Duty |
|------|--------|-------|-------|------|------|------|------|
| `SmallHole` | ≤ 1mm | 5000 | 1 | 80 | 20 | 0 | 1.0 |
| `MediumHole` | 1–3mm | 8000 | 2 | 100 | 30 | 50 | 1.0 |
| `LargeHole` | 3–5mm | 12000 | 3 | 120 | 40 | 100 | 0.9 |
| `ExtraLargeHole` | > 5mm | 15000 | 5 | 150 | 50 | 150 | 0.85 |

`Custom(power, rings, feed, hold, coolDown=0, duty=1.0)` 可手动配置。

### DrillingStrategy

钻孔加工策略枚举（影响孔位访问顺序）：

| 枚举值 | 说明 |
|--------|------|
| `TimeOptimal` | 加工时间最短：纯空间分区 + 分区内 TSP，忽略孔径分组（默认） |
| `QualityOptimal` | 工艺效果优先：按孔径分组，同一种孔径全幅面一次加工完，再加工下一种（组内仍按分区+TSP） |

### DrillPlanner

静态类。将孔位列表优化为最短加工路径（振镜优先聚类 + 分区内 2-opt TSP）。

```csharp
static DrillingTrajectory Plan(
    DrillingPattern pattern,
    double dwellTimeMs = 50.0,       // 单孔停留时间 ms
    double galvoFov = 5.0,           // 振镜半视场 mm（聚类网格尺寸）
    bool galvoFirst = false,         // 振镜优先：2·FOV 网格聚类，簇内全走振镜，仅簇间动平台
    double jumpSpeedPlatform = 500,  // 平台空移速度 mm/s
    double jumpSpeedGalvo = 2000,    // 振镜空移速度 mm/s
    double sampleRate = 1000,        // 采样率 Hz
    DrillingStrategy strategy = DrillingStrategy.TimeOptimal)
```

**振镜优先策略**（`galvoFirst=true`）：以 `2·galvoFov` 为网格尺寸将孔聚类→按簇质心莫顿码（Z-order）排簇→簇内先最近邻贪心再 2-opt 优化（簇孔数 ≤ 300 时）。密度（均孔数/单元）< 4 时回退到 Z-order 或网格最近邻排序。

**返回** `DrillingTrajectory`：`Moves`（`HoleMove` 列表）、`HoleCount`、`TotalDurationMs`、`SampledTrajectory`（用于激光控制的采样轨迹）。

```csharp
foreach (var h in pattern.Holes) h.RecomputeProcessParams(); // 先配工艺参数
var traj = DrillPlanner.Plan(pattern, galvoFov: 5, galvoFirst: true,
    strategy: DrillingStrategy.QualityOptimal);
Console.WriteLine($"{traj.HoleCount:N0} 孔，~{traj.TotalDurationMs/1000:F1}s");
```

---

## Simulation — 动力学仿真

命名空间：`GalvoStage.Core.Simulation`

### StageAxisModel

XY 平台单轴模型（二阶欠阻尼系统 + 扰动 + 噪声）。

| 属性 | 默认 | 说明 |
|------|------|------|
| `BandwidthHz` | 15 | 伺服带宽（Hz） |
| `Damping` | 0.85 | 阻尼比 ζ |
| `MaxVelocity` | 500 | 最大速度（mm/s，0=不限） |
| `DisturbanceAmp` | 0.02 | 正弦扰动幅值（mm，模拟丝杠周期误差） |
| `DisturbanceFreq` | 7 | 扰动频率（Hz） |
| `NoiseAmp` | 0.002 | 随机噪声幅值（mm） |
| `Position` | — | 当前实测位置（只读） |

| 方法 | 说明 |
|------|------|
| `StageAxisModel(int seed)` | 构造，`seed` 决定噪声/扰动相位 |
| `Reset(double pos)` | 复位到指定位置 |
| `Step(double cmd, double dt)` | 推进一周期，返回含扰动的编码器实测位置 |

### GalvoAxisModel

振镜单轴模型（一阶惯性环节 + 视场限幅）。

| 属性 | 默认 | 说明 |
|------|------|------|
| `TimeConstant` | 0.0003 | 时间常数 τ（s，亚毫秒级） |
| `Fov` | 5 | 半视场（±mm） |
| `Position` | — | 当前偏摆（只读） |

| 方法 | 说明 |
|------|------|
| `Reset()` | 复位到 0 |
| `Step(double cmd, double dt)` | 推进一周期，返回限幅后的实际偏摆 |

### LinkageSimulator

联动仿真控制器：整合分解结果与动力学模型，实现实时监控 + 误差方向补偿。

**构造**

```csharp
LinkageSimulator(
    DecomposeResult plan,
    double stageBandwidthHz,   // 平台带宽 Hz
    double stageDamping,       // 平台阻尼比
    double disturbAmp,         // 平台扰动幅值 mm
    double disturbFreq,        // 平台扰动频率 Hz
    double galvoFov,           // 振镜半视场 mm
    double galvoTimeConst)     // 振镜时间常数 s
```

**控制参数**

| 成员 | 类型 | 说明 |
|------|------|------|
| `CompensationEnabled` | `bool` | 是否启用"平台误差→振镜"方向补偿（默认 true） |
| `LeadSamples` | `int` | 平台指令前瞻采样数（构造时按 ≈2ζ/ωn 自动计算，可覆盖） |

**进度控制**

| 成员 | 说明 |
|------|------|
| `Step(int n = 1)` | 推进 n 个控制周期 |
| `Reset()` | 复位仿真与统计 |
| `Index` / `Count` / `Done` / `Dt` | 当前步 / 总步数 / 是否完成 / 周期 |

**实时状态**（只读，`Cur*` 前缀，均为当前周期值）

| 成员 | 说明 |
|------|------|
| `CurStageCmdX/Y` | 平台指令位置 |
| `CurStageActX/Y` | 平台实测位置 |
| `CurGalvoX/Y` | 振镜偏摆 |
| `CurSpotX/Y` | 激光落点 |
| `CurStageErr` | 平台跟随误差（合成，mm） |
| `CurSpotErr` | 落点误差（vs 理想轨迹，mm） |
| `CurLaserOn` | 当前是否出光 |

**历史数组**（长度 = `Count`，用于绘图）：`SpotX/Y`、`StageActX/Y`、`StageErrX/Y`、`SpotError`。

**统计指标**（仅统计激光开的加工段）

| 成员 | 说明 |
|------|------|
| `MaxSpotError` | 最大落点误差（mm） |
| `RmsSpotError` | 落点误差 RMS（mm） |

```csharp
var sim = new LinkageSimulator(plan,
    stageBandwidthHz: 12, stageDamping: 0.85,
    disturbAmp: 0.03, disturbFreq: 7,
    galvoFov: 5, galvoTimeConst: 0.0003)
{ CompensationEnabled = true };

sim.Step(sim.Count);   // 整段运行
Console.WriteLine($"最大误差={sim.MaxSpotError*1000:F1} µm, RMS={sim.RmsSpotError*1000:F1} µm");
```

---

## 完整调用示例

从 DXF 到联动仿真的最小可运行链路（等价于 `tools/SmokeTest`）：

```csharp
using GalvoStage.Core.Dxf;
using GalvoStage.Core.PathPlanning;
using GalvoStage.Core.Simulation;

// 1) 解析 DXF
var polylines = DxfParser.ParseFile("demo.dxf");

// 2) 等时采样（进给 80mm/s，平台空移 500mm/s，振镜空移 2000mm/s，采样率 1kHz）
var traj = PathSampler.Sample(polylines, feedSpeed: 80,
    jumpSpeedPlatform: 500, jumpSpeedGalvo: 2000, sampleRate: 1000);

// 3) 频率分解（自动搜索截止频率，振镜半视场 ±5mm）
var plan = FrequencyDecomposer.DecomposeAuto(traj, galvoFov: 5);
Console.WriteLine($"截止频率={plan.CutoffHz:F2}Hz, 振镜偏摆={plan.MaxGalvoDeviation:F3}mm");

// 4) 联动仿真：对比补偿开/关
foreach (bool comp in new[] { false, true })
{
    var sim = new LinkageSimulator(plan,
        stageBandwidthHz: 12, stageDamping: 0.85,
        disturbAmp: 0.03, disturbFreq: 7,
        galvoFov: 5, galvoTimeConst: 0.0003)
    { CompensationEnabled = comp };

    sim.Step(sim.Count);
    Console.WriteLine($"补偿={(comp ? "开" : "关")}: " +
        $"最大误差={sim.MaxSpotError*1000:F1}µm, RMS={sim.RmsSpotError*1000:F1}µm");
}
```

**典型输出**：

```
截止频率=6.24Hz, 振镜偏摆=3.985mm
补偿=关: 最大误差=632.1µm, RMS=147.5µm
补偿=开: 最大误差= 26.4µm, RMS=  1.4µm
```

---

*本文档与源码同步维护；类型定义见 `src/GalvoStage.Core/` 对应文件。*
