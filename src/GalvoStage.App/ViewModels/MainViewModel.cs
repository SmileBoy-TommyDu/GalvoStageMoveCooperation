using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using GalvoStage.Core.Dxf;
using GalvoStage.Core.Geometry;
using GalvoStage.Core.PathPlanning;
using GalvoStage.Core.Simulation;

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
    public DecomposeResult? Plan { get; private set; }
    public LinkageSimulator? Sim { get; private set; }

    private bool _isRunning;
    public bool IsRunning { get => _isRunning; set => Set(ref _isRunning, value); }

    private string _fileInfo = "未导入图形";
    public string FileInfo { get => _fileInfo; set => Set(ref _fileInfo, value); }

    private string _planInfo = "尚未分解";
    public string PlanInfo { get => _planInfo; set => Set(ref _planInfo, value); }

    private string _realtimeInfo = "";
    public string RealtimeInfo { get => _realtimeInfo; set => Set(ref _realtimeInfo, value); }

    /// <summary>请求重绘（导入/分解/复位后由窗口刷新画布）</summary>
    public event Action? SceneChanged;

    private double _stepAccumulator;

    // ================= 操作 =================

    public void ImportDxf(string path)
    {
        Polylines = DxfParser.ParseFile(path);
        Plan = null; Sim = null; IsRunning = false;

        double totalLen = 0; int ptCount = 0;
        foreach (var pl in Polylines) { totalLen += pl.Length; ptCount += pl.Points.Count; }
        FileInfo = $"{Path.GetFileName(path)}\n轮廓数: {Polylines.Count}   顶点数: {ptCount}\n加工总长: {totalLen:F1} mm";
        PlanInfo = "尚未分解";
        RealtimeInfo = "";
        SceneChanged?.Invoke();
    }

    /// <summary>路径分解：采样 + 频率分解</summary>
    public void Decompose()
    {
        if (Polylines.Count == 0) return;

        var traj = PathSampler.Sample(Polylines, FeedSpeed, RapidSpeed, SampleRate);
        Plan = AutoCutoff
            ? FrequencyDecomposer.DecomposeAuto(traj, GalvoFov)
            : FrequencyDecomposer.Decompose(traj, CutoffHz, GalvoFov);

        if (AutoCutoff) CutoffHz = Math.Round(Plan.CutoffHz, 2);

        RebuildSimulator();

        string fovState = Plan.MaxGalvoDeviation <= GalvoFov ? "√ 在视场内" : "× 超出视场!";
        PlanInfo =
            $"采样点数: {Plan.Count}   加工时长: {traj.Duration:F1} s\n" +
            $"截止频率: {Plan.CutoffHz:F2} Hz\n" +
            $"振镜最大偏摆: {Plan.MaxGalvoDeviation:F3} mm ({fovState})\n" +
            $"平台峰值速度: {Plan.StageMaxVelocity:F1} mm/s\n" +
            $"平台峰值加速度: {Plan.StageMaxAcceleration:F0} mm/s²";
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
            $"进度: {progress,6:F1} %   t = {Sim.Index / SampleRate,7:F2} s\n" +
            $"平台指令: ({Sim.CurStageCmdX,8:F3}, {Sim.CurStageCmdY,8:F3})\n" +
            $"平台实测: ({Sim.CurStageActX,8:F3}, {Sim.CurStageActY,8:F3})\n" +
            $"平台跟随误差: {Sim.CurStageErr * 1000,7:F1} µm\n" +
            $"振镜偏摆: ({Sim.CurGalvoX,7:F3}, {Sim.CurGalvoY,7:F3}) mm\n" +
            $"落点误差: {Sim.CurSpotErr * 1000,7:F1} µm\n" +
            $"― 加工段统计 ―\n" +
            $"最大误差: {Sim.MaxSpotError * 1000,7:F1} µm\n" +
            $"RMS 误差: {Sim.RmsSpotError * 1000,7:F1} µm\n" +
            $"补偿状态: {(Sim.CompensationEnabled ? "已启用 (误差→振镜)" : "已关闭")}";
    }
}
