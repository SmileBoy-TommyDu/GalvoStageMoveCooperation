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

    /// <summary>
    /// 钻孔轨迹 → 等时采样 → 频率分解 → 仿真准备。
    /// 孔间按快移速度插补（激光关），孔位停留 DwellTimeMs（激光开=钻孔）；
    /// 孔数过多时等间距抽样代表子集，避免采样点爆炸。
    /// </summary>
    public void DecomposeDrilling()
    {
        if (DrillingTrajectory == null || DrillingTrajectory.Moves.Count == 0)
        {
            PlanInfo = "⚠️ 请先完成钻孔路径规划";
            return;
        }

        // 仿真孔数上限：2000 孔 × 50ms 停留 @1kHz ≈ 10 万采样点，与现有管线规模一致
        const int MaxSimHoles = 2_000;
        var moves = DrillingTrajectory.Moves;
        string note = "";
        IReadOnlyList<DrillPlanner.HoleMove> simMoves;
        if (moves.Count > MaxSimHoles)
        {
            var picked = new List<DrillPlanner.HoleMove>(MaxSimHoles);
            double stride = moves.Count / (double)MaxSimHoles;
            for (int i = 0; i < MaxSimHoles; i++)
                picked.Add(moves[(int)(i * stride)]);
            simMoves = picked;
            note = $"孔位抽样：{moves.Count:N0} → {picked.Count:N0}（仿真代表子集）\n";
        }
        else simMoves = moves;

        // 等时采样：孔间快移插补（激光关）+ 孔位停留（激光开）
        double dt = 1.0 / SampleRate;
        double rapidStep = RapidSpeed * dt;
        var xs = new List<double>(1 << 17);
        var ys = new List<double>(1 << 17);
        var laser = new List<bool>(1 << 17);

        Vec2 cur = new(simMoves[0].Position.X, simMoves[0].Position.Y);
        foreach (var m in simMoves)
        {
            var target = m.Position;
            double len = cur.DistanceTo(target);
            if (len > 1e-12)
            {
                int steps = Math.Max(1, (int)Math.Ceiling(len / rapidStep));
                for (int s = 1; s <= steps; s++)
                {
                    double t = (double)s / steps;
                    xs.Add(cur.X + (target.X - cur.X) * t);
                    ys.Add(cur.Y + (target.Y - cur.Y) * t);
                    laser.Add(false);
                }
            }
            int dwellN = Math.Max(1, (int)(m.DwellTimeMs / 1000.0 * SampleRate));
            for (int s = 0; s < dwellN; s++)
            {
                xs.Add(target.X); ys.Add(target.Y); laser.Add(true);
            }
            cur = target;
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
        PlanInfo =
            note +
            $"钻孔仿真：{simMoves.Count:N0} 孔   采样点数：{Plan.Count:N0}   时长：{traj.Duration:F1} s\n" +
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
            DrillingInfo = $"✅ G 代码已导出：{n:N0} 孔\n{Path.GetFileName(path)}";
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
            // 对于超大文件，可以使用采样策略
            const int MaxSampleHoles = 100_000;
            List<DrillingPattern.Hole> sampleHoles;
                    
            if (holeCount > MaxSampleHoles)
            {
                Console.WriteLine($"📊 {holeCount:N0} 孔过大，进行降采样...");
                sampleHoles = SampleHoles(pattern, MaxSampleHoles);
                DrillingInfo += $"  → 采样至{sampleHoles.Count:N0}个孔\n";
            }
            else
            {
                sampleHoles = new List<DrillingPattern.Hole>(pattern.Holes);
            }
            
            var samplePattern = new DrillingPattern();
            foreach (var h in sampleHoles)
                samplePattern.Holes.Add(h);
            samplePattern.RecomputeBounds();
            
            DrillingTrajectory = DrillPlanner.Plan(samplePattern, dwellTimeMs);
            DrillingInfo = $"✅ 路径规划完成！\n孔数：{DrillingTrajectory!.HoleCount:N0}\n" +
                          $"预计加工时长：{DrillingTrajectory.TotalDurationMs / 1000:F1} s\n" +
                          "→ 点击“执行路径分解”准备仿真";
        });
        
        SceneChanged?.Invoke();
    }
    
    private static List<DrillingPattern.Hole> SampleHoles(DrillingPattern pattern, int targetCount)
    {
        var result = new List<DrillingPattern.Hole>();
        double step = pattern.Holes.Count / (double)targetCount;
        
        for (int i = 0; i < pattern.Holes.Count; i++)
        {
            if (result.Count >= targetCount) break;
            if (i % (int)step == 0 || i == pattern.Holes.Count - 1)
                result.Add(pattern.Holes[i]);
        }
        
        return result;
    }
    
    /// <summary>生成优化钻孔路径</summary>
    public void PlanDrillingPath(double dwellTimeMs = 50.0)
    {
        if (DrillingPattern == null || DrillingPattern.Holes.Count == 0) return;
        
        var pattern = DrillingPattern;
        DrillingTrajectory = DrillPlanner.Plan(pattern, dwellTimeMs);
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
