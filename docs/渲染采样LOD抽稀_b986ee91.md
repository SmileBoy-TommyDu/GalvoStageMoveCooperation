# 渲染/采样层 LOD 与抽稀方案

## 问题背景
test-panel-800w.dxf 解析后为 836 万轮廓（微型焊盘，圆 r≈0.01mm）/ 1 亿顶点。当前下游三处在此量级不可用：
- `SceneRenderer.DrawScene`：每帧为每条轮廓建一个 SKPath 并绘制全部顶点，无视口裁剪，单帧需数秒。
- `MainWindow.FitView`：导入/双击时在 UI 线程全量扫描 1 亿顶点求包围盒。
- `PathSampler.OrderByNearest`：O(K²) 贪心排序，K=836 万时约 7×10^13 次运算，完全不可行。

设计原则：**轮廓数 ≤ 阈值时行为与现状完全一致（demo.dxf、105-XN 零回归）；超过阈值才启用 LOD/抽稀**。

## 一、几何缓存（新文件 src/GalvoStage.App/Rendering/SceneGeometryCache.cs）
- 新增 `SceneGeometryCache`：为每条轮廓缓存包围盒（`float[] MinX/MinY/MaxX/MaxY` 扁平数组，约 134MB@836万条），并保存全局包围盒与总顶点数、总长度。
- `Build(List<PathPolyline>)` 用 `Parallel.For` 一次遍历完成（顺带算总长/顶点数，替代 ImportDxf 里的串行统计循环）。
- `MainViewModel` 新增 `GeometryCache` 属性，`ImportDxf` 解析后立即构建。

## 二、渲染 LOD（SceneRenderer.cs + MainWindow.xaml.cs）
- `FitView` 改用 `GeometryCache` 的全局包围盒，O(1)，删除全量双重循环。
- `DrawScene` 原始图形绘制改为两条路径：
  - 轮廓数 ≤ 50,000：维持现有逐条 `DrawPolyline`（全保真，零回归）。
  - 超过 50,000：遍历包围盒数组做**视口裁剪**（视口外直接跳过）；视口内轮廓按屏幕尺寸分级——
    - 屏幕尺寸 < 2px：仅收集包围盒中心点，最后用 `canvas.DrawPoints(SKPointMode.Points, ...)` 批量绘制（点云）；
    - ≥ 2px：完整绘制，但合并到共享 SKPath 分批 flush，且设全路径预算 20,000 条，超预算部分退化为点。
  - 点缓冲用可复用的 `List<SKPoint>`/数组，避免每帧大分配。
- 效果：缩小看整板时为点云（视觉上与亚像素焊盘一致），放大后自动显示真实轮廓；帧时间由视口内容决定而非总量。

## 三、采样层抽稀与排序（PathSampler.cs + MainViewModel.cs）
- `OrderByNearest` 增加网格加速版本：轮廓数 > 5,000 时，把各轮廓端点挂入均匀空间网格，贪心最近邻查询改为从当前网格单元环形向外扩展，复杂度近似 O(K)；小数据保留原实现。
- 新增 `PathSampler.Decimate(polylines, maxCount)`：按 √maxCount×√maxCount 空间网格分桶、逐桶轮流取样至 maxCount 条，保证空间均匀覆盖的代表性子集。
- `MainViewModel.Decompose`：轮廓数 > 20,000 时先 `Decimate` 到 20,000 条再采样/分解/仿真，`PlanInfo` 首行明示「轮廓抽稀: 8,363,616 → 20,000（仿真代表子集）」；未超阈值行为不变。

## 四、验证
- `dotnet build GalvoStageLink.sln -c Release` 0 错误。
- 启动应用导入 test-panel-800w.dxf：导入后自动 Fit 即时完成；平移/缩放流畅（整板点云、放大出轮廓）；「路径分解」数秒内完成且仿真可运行。
- 回归：demo.dxf（9 轮廓）与 105-XN（9,685 轮廓）渲染、分解、仿真行为与现状一致。

## 假设与取舍
- 全量 836 万焊盘的真实加工轨迹采样在演示工具中无意义（轨迹点数亿级、仿真数组内存爆炸），故仿真采用空间均匀抽稀子集，并在界面明示；渲染层则始终保留全量数据，仅按视口/像素粒度决定绘制方式。
- 阈值（50,000 / 20,000 / 2px / 5,000）定义为 `SceneRenderer`/`PathSampler` 中的常量，便于后续调整。