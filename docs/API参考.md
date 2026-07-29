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
| `Length` | `double` | 折线总长（闭合时含收尾段） |

---

## Dxf — DXF 解析

命名空间：`GalvoStage.Core.Dxf`

### DxfParser

静态类。轻量 ASCII DXF 解析器，读取 `ENTITIES` 段并将实体统一细分为 `PathPolyline`。

| 方法 | 签名 | 说明 |
|------|------|------|
| `ParseFile` | `static List<PathPolyline> ParseFile(string path)` | 从文件路径解析 |
| `Parse` | `static List<PathPolyline> Parse(TextReader reader)` | 从文本流解析 |

| 常量 | 值 | 说明 |
|------|-----|------|
| `ChordTolerance` | `0.01` | 圆弧细分弦高误差（mm） |

**支持实体**：`LINE`、`CIRCLE`、`ARC`、`LWPOLYLINE`（含凸度 bulge）、`POLYLINE`/`VERTEX`、`ELLIPSE`、`SPLINE`（NURBS/拟合点）。

```csharp
List<PathPolyline> shapes = DxfParser.ParseFile(@"C:\part.dxf");
Console.WriteLine($"轮廓数={shapes.Count}, 总长={shapes.Sum(p => p.Length):F1} mm");
```

> **注**：仅支持 ASCII DXF；不支持二进制 DXF、块引用（INSERT）展开。未识别实体会被忽略。

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
    double feedSpeed,     // 进给速度 mm/s（轮廓，激光开）
    double rapidSpeed,    // 快移速度 mm/s（空程，激光关）
    double sampleRate)    // 采样率 Hz
```

**内部处理**：① 贪心最近邻排序（可整体反向折线）减少空程；② 空程/轮廓按各自速度等距插补；③ 跨段相位连续。

```csharp
var traj = PathSampler.Sample(shapes, feedSpeed: 80, rapidSpeed: 300, sampleRate: 1000);
```

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

// 2) 等时采样（进给 80mm/s，快移 300mm/s，采样率 1kHz）
var traj = PathSampler.Sample(polylines, feedSpeed: 80, rapidSpeed: 300, sampleRate: 1000);

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
