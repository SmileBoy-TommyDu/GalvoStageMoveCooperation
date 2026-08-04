using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using GalvoStage.App.Rendering;
using GalvoStage.Core.Dxf;
using GalvoStage.Core.Geometry;
using GalvoStage.Core.PathPlanning;
using GalvoStage.Core.Simulation;
using GalvoStage.Core.Geometry.Drilling;
using GalvoStage.Core.Drilling;

namespace GalvoStage.App.ViewModels;

/// <summary>主视图模型：参数、DXF→分解→仿真 流水线状态</summary>
public sealed class MainViewModel : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;
    private void Raise([CallerMemberName] string? name = null)
        => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

    private bool Set<T>(ref T field, T value, [CallerMemberName] string? name = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value)) return false;
        field = value; Raise(name); return true;
    }

    // ================= 工艺参数 =================
    private double _feedSpeed = 80;
    public double FeedSpeed { get => _feedSpeed; set => Set(ref _feedSpeed, value); }

    private double _rapidSpeed = 300;
    public double RapidSpeed { get => _rapidSpeed; set => Set(ref _rapidSpeed, value); }

    private double _sampleRate = 1000;
    public double SampleRate { get => _sampleRate; set => Set(ref _sampleRate, value); }

    // ================= 频率分解参数 =================
    private bool _autoCutoff = true;
    public bool AutoCutoff { get => _autoCutoff; set => Set(ref _autoCutoff, value); }

    private double _cutoffHz = 6;
    public double CutoffHz { get => _cutoffHz; set => Set(ref _cutoffHz, value); }

    private double _galvoFov = 5;
    public double GalvoFov { get => _galvoFov; set => Set(ref _galvoFov, value); }

    // ================= 仿真参数 =================
    private double _stageBandwidth = 12;
    public double StageBandwidth { get => _stageBandwidth; set => Set(ref _stageBandwidth, value); }

    private double _stageDamping = 0.85;
    public double StageDamping { get => _stageDamping; set => Set(ref _stageDamping, value); }

    private double _disturbAmp = 0.03;
    public double DisturbAmp { get => _disturbAmp; set => Set(ref _disturbAmp, value); }

    private double _disturbFreq = 7;
    public double DisturbFreq { get => _disturbFreq; set => Set(ref _disturbFreq, value); }

    private double _galvoTauMs = 0.3;
    public double GalvoTauMs { get => _galvoTauMs; set => Set(ref _galvoTauMs, value); }

    private double _simSpeed = 1.0;
    public double SimSpeed { get => _simSpeed; set => Set(ref _simSpeed, value); }

    private bool _compensationOn = true;
    public bool CompensationOn
    {
        get => _compensationOn;
        set { if (Set(ref _compensationOn, value) && Sim != null) Sim.CompensationEnabled = value; }
    }

    // ================= 流水线状态 =================
    public List<PathPolyline> Polylines { get; private set; } = new();
    /// <summary>轮廓包围盒/统计缓存（导入时并行构建，供 LOD 渲染与 FitView 使用）</summary>
    public SceneGeometryCache? GeometryCache { get; private set; }
    public DecomposeResult? Plan { get; private set; }
    public LinkageSimulator? Sim { get; private set; }
    
    // ================= PCB 钻孔模式状态 =================
    public GalvoStage.Core.Geometry.Drilling.DrillingPattern? DrillingPattern { get; private set; }
    public DrillPlanner.DrillingTrajectory? DrillingTrajectory { get; private set; }

    private bool _isRunning;
    public bool IsRunning { get => _isRunning; set => Set(ref _isRunning, value); }

    private string _fileInfo = "未导入图形";
    public string FileInfo { get => _fileInfo; set => Set(ref _fileInfo, value); }

    private string _planInfo = "尚未分解";
    public string PlanInfo { get => _planInfo; set => Set(ref _planInfo, value); }

    private string _realtimeInfo = "";
    public string RealtimeInfo { get => _realtimeInfo; set => Set(ref _realtimeInfo, value); }
    
    /// <summary>钻孔 DXF 导入状态信息</summary>
    private string _drillingInfo = "";
    public string DrillingInfo { get => _drillingInfo; set => Set(ref _drillingInfo, value); }
    
    /// <summary>请求重绘（导入/分解/复位后由窗口刷新画布）</summary>
    public event Action? SceneChanged;

    private double _stepAccumulator;
    
    // ================= 操作 =================

    public void ImportDxf(string path)
    {
        Polylines = DxfParser.ParseFile(path);
        Plan = null; Sim = null; IsRunning = false;
        // 清理上一次钻孔导入残留（避免先导入钻孔 DXF 再导入普通 DXF 时，钻孔点云仍留在画布）
        DrillingPattern = null;
        DrillingTrajectory = null;
        DrillingInfo = "";

        CenterPolylinesAtOrigin(Polylines);
        GeometryCache = SceneGeometryCache.Build(Polylines);
        FileInfo = $"{Path.GetFileName(path)}\n轮廓数：{Polylines.Count:N0}   顶点数：{GeometryCache.VertexCount:N0}\n加工总长：{GeometryCache.TotalLength:F1} mm\n{FormatLayers(GeometryCache.LayerCounts)}";
        PlanInfo = "尚未分解";
        RealtimeInfo = "";
        SceneChanged?.Invoke();
    }

    /// <summary>图层统计摘要：按轮廓数降序，最多列出 5 个图层名</summary>
    private static string FormatLayers(IReadOnlyDictionary<string, int> layerCounts)
    {
        if (layerCounts.Count == 0) return "图层：无";
        var top = layerCounts.OrderByDescending(kv => kv.Value).Take(5)
            .Select(kv => $"{kv.Key}({kv.Value:N0})");
        string more = layerCounts.Count > 5 ? " …" : "";
        return $"图层：{layerCounts.Count} 个  {string.Join("  ", top)}{more}";
    }

    /// <summary>将轮廓集合整体平移，使包围盒中心落在世界原点 (0,0)</summary>
    private static void CenterPolylinesAtOrigin(List<PathPolyline> polylines)
    {
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        foreach (var pl in polylines)
        {
            var pts = pl.Points;
            for (int i = 0; i < pts.Count; i++)
            {
                var p = pts[i];
                if (p.X < minX) minX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.X > maxX) maxX = p.X;
                if (p.Y > maxY) maxY = p.Y;
            }
        }
        if (minX > maxX) return; // 无有效顶点

        double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
        if (Math.Abs(cx) < 1e-9 && Math.Abs(cy) < 1e-9) return; // 已居中

        foreach (var pl in polylines)
        {
            var pts = pl.Points;
            for (int i = 0; i < pts.Count; i++)
                pts[i] = new Vec2(pts[i].X - cx, pts[i].Y - cy);
        }
    }

    /// <summary>将钻孔点集整体平移，使包围盒中心落在世界原点 (0,0)</summary>
    private static void CenterDrillingAtOrigin(DrillingPattern pattern)
    {
        if (pattern.Bounds is not { } b) return;
        double cx = (b.MinX + b.MaxX) / 2, cy = (b.MinY + b.MaxY) / 2;
        if (Math.Abs(cx) < 1e-9 && Math.Abs(cy) < 1e-9) return; // 已居中

        var holes = pattern.Holes;
        for (int i = 0; i < holes.Count; i++)
        {
            var h = holes[i];
            h.X -= cx; h.Y -= cy;
            holes[i] = h;
        }
        pattern.RecomputeBounds();
    }

    /// <summary>
    /// 路径分解：采样 + 频率分解。为兼顾“交互性能”与“加工完整性”，采用两阶段策略：
    ///   ① 参数估计：轮廓数超阀时，仅在空间均匀抽稀的代表子集上跑 DecomposeAuto（昂贵的二分迭代只跑子集），仅用于求截止频率；
    ///   ② 加工指令：对全部轮廓采样后，用固定截止频率做单次全量分解，保证每一条轮廓都被加工、不丢弃。
    /// 抽稀只影响“估参”这一步，绝不影响最终运动指令。
    /// </summary>
    public void Decompose()
    {
        if (Polylines.Count == 0) return;

        string note;
        if (AutoCutoff)
        {
            if (Polylines.Count > PathSampler.MaxSampleContours)
            {
                // 阶段①：仅在代表子集上自动搜索截止频率（昂贵的 18 次 filtfilt 迭代只跑子集）
                var subset = PathSampler.Decimate(Polylines, PathSampler.MaxSampleContours);
                var subsetTraj = PathSampler.Sample(subset, FeedSpeed, RapidSpeed, SampleRate);
                double cutoff = FrequencyDecomposer.DecomposeAuto(subsetTraj, GalvoFov).CutoffHz;

                // 阶段②：全量采样 + 固定截止频率单次分解（不丢任何轮廓）
                var traj = PathSampler.Sample(Polylines, FeedSpeed, RapidSpeed, SampleRate);
                Plan = FrequencyDecomposer.Decompose(traj, cutoff, GalvoFov);
                note = $"截止频率在 {subset.Count:N0}/{Polylines.Count:N0} 代表子集上估得，全量分解\n";
            }
            else
            {
                var traj = PathSampler.Sample(Polylines, FeedSpeed, RapidSpeed, SampleRate);
                Plan = FrequencyDecomposer.DecomposeAuto(traj, GalvoFov);
                note = "";
            }
            CutoffHz = Math.Round(Plan.CutoffHz, 2);
        }
        else
        {
            // 手动截止频率：无需估参，直接全量采样 + 单次分解
            var traj = PathSampler.Sample(Polylines, FeedSpeed, RapidSpeed, SampleRate);
            Plan = FrequencyDecomposer.Decompose(traj, CutoffHz, GalvoFov);
            note = "";
        }

        RebuildSimulator();

        string fovState = Plan.MaxGalvoDeviation <= GalvoFov ? "√ 在视场内" : "× 超出视场!";
        PlanInfo =
            note +
            $"加工轮廓：{Polylines.Count:N0} 条（全量，无丢弃）\n" +
            $"采样点数：{Plan.Count}   加工时长：{Plan.Raw.Duration:F1} s\n" +
            $"截止频率：{Plan.CutoffHz:F2} Hz\n" +
            $"振镜最大偏摆：{Plan.MaxGalvoDeviation:F3} mm ({fovState})\n" +
            $"平台峰值速度：{Plan.StageMaxVelocity:F1} mm/s\n" +
            $"平台峰值加速度：{Plan.StageMaxAcceleration:F0} mm/s²";
        SceneChanged?.Invoke();
    }

    public void RebuildSimulator()
    {
        if (Plan == null) return;
        Sim = new LinkageSimulator(Plan,
            StageBandwidth, StageDamping,
            DisturbAmp, DisturbFreq,
            GalvoFov, Math.Max(GalvoTauMs, 0.05) / 1000.0)
        { CompensationEnabled = CompensationOn };
        _stepAccumulator = 0;
        IsRunning = false;
        UpdateRealtimeInfo();
        SceneChanged?.Invoke();
    }

    /// <summary>由渲染循环驱动：按真实流逝时间推进仿真</summary>
    public void Advance(double elapsedSeconds)
    {
        if (!IsRunning || Sim == null || Plan == null) return;
        _stepAccumulator += elapsedSeconds * SampleRate * SimSpeed;
        int steps = (int)_stepAccumulator;
        if (steps <= 0) return;
        _stepAccumulator -= steps;
        Sim.Step(Math.Min(steps, 20000));
        if (Sim.Done) IsRunning = false;
        UpdateRealtimeInfo();
    }

    public void UpdateRealtimeInfo()
    {
        if (Sim == null || Plan == null) { RealtimeInfo = ""; return; }
        double progress = Sim.Count > 0 ? 100.0 * Sim.Index / Sim.Count : 0;
        RealtimeInfo =
            $"进度：{progress,6:F1} %   t = {Sim.Index / SampleRate,7:F2} s\n" +
            $"平台指令：({Sim.CurStageCmdX,8:F3}, {Sim.CurStageCmdY,8:F3})\n" +
            $"平台实测：({Sim.CurStageActX,8:F3}, {Sim.CurStageActY,8:F3})\n" +
            $"平台跟随误差：{Sim.CurStageErr * 1000,7:F1} µm\n" +
            $"振镜偏摆：({Sim.CurGalvoX,7:F3}, {Sim.CurGalvoY,7:F3}) mm\n" +
            $"落点误差：{Sim.CurSpotErr * 1000,7:F1} µm\n" +
            $"― 加工段统计 ―\n" +
            $"最大误差：{Sim.MaxSpotError * 1000,7:F1} µm\n" +
            $"RMS 误差：{Sim.RmsSpotError * 1000,7:F1} µm\n" +
            $"补偿状态：{(Sim.CompensationEnabled ? "已启用 (误差→振镜)" : "已关闭")}";
    }
    
    /// <summary>导入 PCB 钻孔 DXF（CIRCLE 实体）；layerFilter 非空时仅导入指定图层（分批导入降内存）</summary>
    public void ImportDrillingFile(string path, string? layerFilter = null)
    {
        try
        {
            Polylines.Clear(); GeometryCache = null; Plan = null; Sim = null; DrillingTrajectory = null;
            
            var pattern = DrillingDxfParser.ParseFile(path, layerFilter);
            CenterDrillingAtOrigin(pattern);
            DrillingPattern = pattern;
            
            string layerInfo = pattern.LayerCounts.Count == 0 
                ? "" 
                : $"\n{FormatLayersForDrilling(pattern.LayerCounts)}";
            string diaInfo = FormatDiameters(pattern.DiameterCounts);
            string filterNote = layerFilter == null ? "" : $"\n图层过滤：{layerFilter}";
                
            FileInfo = $"{Path.GetFileName(path)}\n钻孔点数：{pattern.Holes.Count:N0}{layerInfo}\n{diaInfo}{filterNote}\n包围盒：{pattern.Bounds?.MinX:F2}×{pattern.Bounds?.MinY:F2} ~ {pattern.Bounds?.MaxX:F2}×{pattern.Bounds?.MaxY:F2} mm";
            PlanInfo = "尚未规划路径";
            DrillingInfo = $"已导入 {pattern.Holes.Count:N0} 个孔";
            RealtimeInfo = "";
            SceneChanged?.Invoke();
        }
        catch (Exception ex)
        {
            Console.WriteLine($"DXF 解析失败：{ex.Message}");
            DrillingInfo = $"❌ 错误：{ex.Message}";
        }
    }

    // ---------------- 双模式分离加工（混合特征） ----------------

    /// <summary>
    /// 双模式分离导入：一次解析同时提取折线特征（轮廓）与钻孔特征（CIRCLE），
    /// 两份数据并存，可分别走折线链路（频域分解）与钻孔链路（空间聚类）独立规划。
    /// 对应 09 号文档“双模式分离加工”方案阶段①。
    /// </summary>
    public void ImportMixed(string path)
    {
        try
        {
            var parsed = DxfParser.ParseFileMixed(path);

            Polylines = parsed.Polylines;
            DrillingPattern = parsed.DrillingHoles;
            Plan = null; Sim = null; IsRunning = false;
            DrillingTrajectory = null;

            // ① 先计算折线原始包围盒中心（居中前），作为两链路坐标对齐的公共基准
            double origMinX = double.MaxValue, origMinY = double.MaxValue;
            double origMaxX = double.MinValue, origMaxY = double.MinValue;
            foreach (var pl in Polylines)
            {
                var pts = pl.Points;
                for (int i = 0; i < pts.Count; i++)
                {
                    var p = pts[i];
                    if (p.X < origMinX) origMinX = p.X;
                    if (p.Y < origMinY) origMinY = p.Y;
                    if (p.X > origMaxX) origMaxX = p.X;
                    if (p.Y > origMaxY) origMaxY = p.Y;
                }
            }
            double origCx = (origMinX + origMaxX) / 2;
            double origCy = (origMinY + origMaxY) / 2;

            // ② 折线特征居中（按原始包围盒中心平移）
            CenterPolylinesAtOrigin(Polylines);
            GeometryCache = SceneGeometryCache.Build(Polylines);

            // ③ 钻孔特征按同一平移量对齐（保证两链路坐标系一致）
            //   关键：必须用“折线居中前”的原始中心，而非居中后的中心（后者≈(0,0)，会导致钻孔未平移）
            if (DrillingPattern.Holes.Count > 0 && origMinX <= origMaxX)
            {
                var holes = DrillingPattern.Holes;
                for (int i = 0; i < holes.Count; i++)
                {
                    var h = holes[i];
                    h.X -= origCx;
                    h.Y -= origCy;
                    holes[i] = h;
                }
                DrillingPattern.RecomputeBounds();
            }

            string contourInfo = $"轮廓数：{Polylines.Count:N0}  顶点数：{GeometryCache?.VertexCount ?? 0:N0}";
            string holeInfo = DrillingPattern.Holes.Count > 0
                ? $"\n钻孔数：{DrillingPattern.Holes.Count:N0}（CIRCLE {parsed.CircleCount:N0}）"
                : "\n钻孔数：0";
            FileInfo = $"{Path.GetFileName(path)}\n{contourInfo}{holeInfo}\n加工总长：{GeometryCache?.TotalLength:F1} mm\n{FormatLayers(GeometryCache?.LayerCounts ?? new Dictionary<string, int>())}";
            PlanInfo = "尚未分解（双模式：折线 + 钻孔）";
            DrillingInfo = DrillingPattern.Holes.Count > 0
                ? $"已导入 {DrillingPattern.Holes.Count:N0} 个孔"
                : "";
            RealtimeInfo = "";
            SceneChanged?.Invoke();
        }
        catch (Exception ex)
        {
            Console.WriteLine($"DXF 混合解析失败：{ex.Message}");
            DrillingInfo = $"❌ 错误：{ex.Message}";
        }
    }

    /// <summary>
    /// 双模式分离分解：折线链路与钻孔链路独立规划，互不干扰。
    /// 折线链路：等时采样 → 频域分解 → 生成 Plan/Sim；
    /// 钻孔链路：振镜优先聚类 → 环切轨迹 → 生成 DrillingTrajectory。
    /// </summary>
    public void DecomposeBoth()
    {
        // 折线链路
        if (Polylines.Count > 0)
        {
            Decompose();
        }
        // 钻孔链路
        if (DrillingPattern != null && DrillingPattern.Holes.Count > 0)
        {
            // 钻孔路径规划（振镜优先）
            DrillingTrajectory = DrillPlanner.Plan(DrillingPattern, dwellTimeMs: 50.0,
                galvoFov: GalvoFov, galvoFirst: true);
            PlanInfo += $"\n\n[钻孔链路] 规划完成：{DrillingTrajectory.Moves.Count:N0} 个孔位移动";
            DrillingInfo = $"已导入 {DrillingPattern.Holes.Count:N0} 个孔 → 已规划路径";
        }
    }

    /// <summary>
    /// 钻孔轨迹 → 等时采样 → 频率分解 → 仿真准备（激光钻孔）。
    /// 孔间按快移速度插补（激光关）；到达孔位后按孔径做<b>环切 trepanning</b>：
    /// 以孔半径沿圆周走 laser on，圈数由停留时间/单圈时间决定——由此<b>体现不同孔径的加工轨迹</b>。
    /// 孔径未知或过小（半径 &lt; 一个进给步距）时回退为原地点钻停留。
    /// 孔数过多时等间距抽样代表子集，避免采样点爆炸。
    /// </summary>
    public void DecomposeDrilling()
    {
        if (DrillingTrajectory == null || DrillingTrajectory.Moves.Count == 0)
        {
            PlanInfo = "⚠️ 请先完成钻孔路径规划";
            return;
        }

        // 仿真孔数上限：2000 孔 × 50ms 停留 @1kHz ≈ 10 万采样点，与现有管线规模一致。
        // 注意：抽样只用于“仿真预览”，全部 moves 已在 DrillingTrajectory 中，G 代码导出/加工不受影响。
        const int MaxSimHoles = 2_000;
        var moves = DrillingTrajectory.Moves;
        string note = "";
        IReadOnlyList<DrillPlanner.HoleMove> simMoves;
        if (moves.Count > MaxSimHoles)
        {
            simMoves = SampleUniform(moves, MaxSimHoles);
            note = $"孔位抽样：{moves.Count:N0} → {simMoves.Count:N0}（仅仿真预览；全部 {moves.Count:N0} 孔仍将导出/加工）\n";
        }
        else simMoves = moves;

        // 等时采样：孔间快移插补（激光关）+ 按孔径环切（激光开=钻孔）
        double dt = 1.0 / SampleRate;
        double rapidStep = RapidSpeed * dt;
        double feedStep = FeedSpeed * dt;
        var xs = new List<double>(1 << 17);
        var ys = new List<double>(1 << 17);
        var laser = new List<bool>(1 << 17);
        int trepanCount = 0, pointCount = 0;

        Vec2 cur = new(simMoves[0].Position.X, simMoves[0].Position.Y);
        foreach (var m in simMoves)
        {
            double cx = m.Position.X, cy = m.Position.Y;
            double r = m.Diameter * 0.5;
            bool trepan = m.Diameter > 0 && r >= feedStep;   // 半径够大才能环切成圆

            // 进刀点：环切起于圆周 (cx+r, cy)，点钻则为圆心
            Vec2 entry = trepan ? new Vec2(cx + r, cy) : new Vec2(cx, cy);

            // 空移到进刀点（激光关）
            double len = cur.DistanceTo(entry);
            if (len > 1e-12)
            {
                int steps = Math.Max(1, (int)Math.Ceiling(len / rapidStep));
                for (int s = 1; s <= steps; s++)
                {
                    double t = (double)s / steps;
                    xs.Add(cur.X + (entry.X - cur.X) * t);
                    ys.Add(cur.Y + (entry.Y - cur.Y) * t);
                    laser.Add(false);
                }
            }

            if (trepan)
            {
                // 环切：沿孔径圆周走 laser on；圈数 = 停留时间 / 单圈时间（体现孔径 + 加工量）
                double circumference = 2 * Math.PI * r;
                int ptsPerLoop = Math.Max(8, (int)Math.Ceiling(circumference / feedStep));
                double loopTimeMs = circumference / FeedSpeed * 1000.0;
                int loops = Math.Max(1, (int)Math.Round(m.DwellTimeMs / loopTimeMs));
                int totalPts = ptsPerLoop * loops;
                for (int k = 1; k <= totalPts; k++)
                {
                    double ang = 2 * Math.PI * (k / (double)ptsPerLoop);
                    xs.Add(cx + r * Math.Cos(ang));
                    ys.Add(cy + r * Math.Sin(ang));
                    laser.Add(true);
                }
                cur = entry;   // 环切结束回到进刀点
                trepanCount++;
            }
            else
            {
                // 点钻（孔径未知或过小）：原地停留
                int dwellN = Math.Max(1, (int)(m.DwellTimeMs / 1000.0 * SampleRate));
                for (int s = 0; s < dwellN; s++)
                {
                    xs.Add(cx); ys.Add(cy); laser.Add(true);
                }
                cur = new Vec2(cx, cy);
                pointCount++;
            }
        }

        var traj = new SampledTrajectory
        {
            SampleRate = SampleRate,
            X = xs.ToArray(),
            Y = ys.ToArray(),
            LaserOn = laser.ToArray()
        };

        Plan = AutoCutoff
            ? FrequencyDecomposer.DecomposeAuto(traj, GalvoFov)
            : FrequencyDecomposer.Decompose(traj, CutoffHz, GalvoFov);
        if (AutoCutoff) CutoffHz = Math.Round(Plan.CutoffHz, 2);

        RebuildSimulator();

        string fovState = Plan.MaxGalvoDeviation <= GalvoFov ? "√ 在视场内" : "× 超出视场!";
        // 逐孔动力学预检（对全量孔，非仅预览子集）
        string precheck = PrecheckDrillDynamics(moves);
        string precheckLine = precheck.Length > 0 ? precheck : "逐孔预检：✅ 全部孔径在当前进给下可行\n";
        PlanInfo =
            note +
            precheckLine +
            $"钻孔仿真预览：{simMoves.Count:N0} 孔 / 全量加工 {moves.Count:N0} 孔   采样点数：{Plan.Count:N0}   时长：{traj.Duration:F1} s\n" +
            $"加工方式（预览子集）：环切 {trepanCount:N0} 孔 / 点钻 {pointCount:N0} 孔\n" +
            $"截止频率：{Plan.CutoffHz:F2} Hz\n" +
            $"振镜最大偏摆：{Plan.MaxGalvoDeviation:F3} mm ({fovState})\n" +
            $"平台峰值速度：{Plan.StageMaxVelocity:F1} mm/s\n" +
            $"平台峰值加速度：{Plan.StageMaxAcceleration:F0} mm/s²";
        DrillingInfo += "\n✅ 仿真已准备，点击“开始仿真”";
        SceneChanged?.Invoke();
    }

    /// <summary>孔径统计摘要：按孔数降序，最多列出 5 种孔径</summary>
    private static string FormatDiameters(IReadOnlyDictionary<double, int> diameterCounts)
    {
        if (diameterCounts.Count == 0) return "孔径：无";
        if (diameterCounts.Count == 1 && diameterCounts.ContainsKey(0)) return "孔径：未知";
        var top = diameterCounts.OrderByDescending(kv => kv.Value).Take(5)
            .Select(kv => kv.Key > 0 ? $"Ø{kv.Key:F3}({kv.Value:N0})" : $"未知({kv.Value:N0})");
        string more = diameterCounts.Count > 5 ? " …" : "";
        return $"孔径：{diameterCounts.Count} 种  {string.Join("  ", top)}{more}";
    }

    /// <summary>最近一次逐孔动力学预检的结构化结果（含环切频率/向心加速度统计），供 UI 与其它消费者读取。</summary>
    public DrillDynamicsReport? LastDrillDynamics { get; private set; }

    /// <summary>
    /// 逐孔动力学预检（对齐 docs/06 §7.4）：委托 <see cref="DrillDynamicsPrecheck"/> 按环切频率
    /// f=v/(2πr) 与向心加速度 a=v²/r 估算，找出在当前进给下平台跟随能力不足、无法完美加工的孔，
    /// 缓存结构化结果到 <see cref="LastDrillDynamics"/> 并格式化为告警文本。全部可行返回空串。
    /// </summary>
    private string PrecheckDrillDynamics(IReadOnlyList<DrillPlanner.HoleMove> moves)
    {
        var report = DrillDynamicsPrecheck.Evaluate(moves, FeedSpeed, StageBandwidth, SampleRate, GalvoFov);
        LastDrillDynamics = report;
        if (report.AllFeasible) return "";

        var sb = new System.Text.StringBuilder();
        sb.Append($"⚠️ 逐孔预检：{report.OffenderKindCount} 种孔径在进给 {FeedSpeed:F0} mm/s 下超平台跟随能力（带宽 {StageBandwidth:F1}Hz）\n");
        foreach (var o in report.Offenders)
            sb.Append($"   Ø{o.Diameter:F3}×{o.Count:N0}：f={o.FrequencyHz:F1}Hz(>{report.FrequencyCapHz:F1}) a={o.AccelMmPerS2:F0}mm/s² → 建议进给≤{o.SuggestedFeed:F0} mm/s\n");
        return sb.ToString();
    }

    /// <summary>导出已规划的钻孔轨迹为 G 代码（按孔径分组换刀）</summary>
    public bool ExportGCode(string path)
    {
        if (DrillingTrajectory == null || DrillingTrajectory.Moves.Count == 0)
        {
            DrillingInfo = "⚠️ 请先完成路径规划再导出 G 代码";
            return false;
        }
        try
        {
            int n = GCodeExporter.Export(DrillingTrajectory, path);
            DrillingInfo = $"✅ G 代码已导出：{n:N0} 孔（全量）\n{Path.GetFileName(path)}";
            return true;
        }
        catch (Exception ex)
        {
            DrillingInfo = $"❌ 导出失败：{ex.Message}";
            return false;
        }
    }
    
    /// <summary>生成优化钻孔路径（异步版本，用于大文件）</summary>
    public async Task PlanDrillingPathAsync(double dwellTimeMs = 50.0)
    {
        if (DrillingPattern == null || DrillingPattern.Holes.Count == 0) 
        {
            DrillingInfo = "⚠️ 无钻孔点可规划";
            return;
        }
        
        var pattern = DrillingPattern;
        int holeCount = pattern.Holes.Count;
        
        DrillingInfo = $"正在规划路径... ({holeCount:N0} 个孔)";
        await Task.Run(() =>
        {
            // 全量规划：所有孔都进入 DrillingTrajectory（G 代码导出/加工的数据源），绝不丢弃。
            // 振镜优先策略（galvoFirst=true）：以 2·FOV 为网格尺寸将孔聚类到簇，簇内全走振镜，
            // 仅簇间才动平台。百万孔 → 几千个簇 → 平台动几千次（而非百万次），大幅节约加工时间。
            // 抽样只发生在仿真预览阶段（DecomposeDrilling），不影响加工指令——对齐激光链路的两阶段策略。
            DrillingTrajectory = DrillPlanner.Plan(pattern, dwellTimeMs, GalvoFov, galvoFirst: true);
            DrillingInfo = $"✅ 路径规划完成！（振镜优先）\n孔数：{DrillingTrajectory!.HoleCount:N0}（全量，无丢弃）\n" +
                          $"预计加工时长：{DrillingTrajectory.TotalDurationMs / 1000:F1} s\n" +
                          "→ 点击“执行路径分解”准备仿真";
        });
        
        SceneChanged?.Invoke();
    }
    
    /// <summary>
    /// 均匀步长抽样：从有序列表中等间距挑选至多 targetCount 个元素。
    /// <b>仅用于仿真预览子集</b>，绝不影响 DrillingTrajectory / G 代码导出的加工数据（全量孔已在规划阶段落地）。
    /// </summary>
    private static List<T> SampleUniform<T>(IReadOnlyList<T> items, int targetCount)
    {
        if (items.Count <= targetCount)
            return new List<T>(items);

        var result = new List<T>(targetCount);
        double stride = items.Count / (double)targetCount;
        for (int i = 0; i < targetCount; i++)
            result.Add(items[(int)(i * stride)]);
        return result;
    }
    
    /// <summary>生成优化钻孔路径</summary>
    public void PlanDrillingPath(double dwellTimeMs = 50.0)
    {
        if (DrillingPattern == null || DrillingPattern.Holes.Count == 0) return;
        
        var pattern = DrillingPattern;
        // 振镜优先策略：簇内全走振镜，仅簇间动平台
        DrillingTrajectory = DrillPlanner.Plan(pattern, dwellTimeMs, GalvoFov, galvoFirst: true);
        PlanInfo = $"孔数：{DrillingTrajectory.HoleCount:N0}\n{DrillingTrajectory}";
        SceneChanged?.Invoke();
    }
    
    private static string FormatLayersForDrilling(IReadOnlyDictionary<string, int> layerCounts)
    {
        if (layerCounts.Count == 0) return "图层：无";
        var top = layerCounts.OrderByDescending(kv => kv.Value).Take(5)
            .Select(kv => $"{kv.Key}({kv.Value:N0})");
        string more = layerCounts.Count > 5 ? " …" : "";
        return $"图层：{layerCounts.Count} 个  {string.Join("  ", top)}{more}";
    }
}
