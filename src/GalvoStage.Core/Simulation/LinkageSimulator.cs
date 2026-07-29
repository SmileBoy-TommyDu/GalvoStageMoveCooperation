using System;
using GalvoStage.Core.PathPlanning;

namespace GalvoStage.Core.Simulation;

/// <summary>
/// 振镜 + XY 平台联动仿真控制器（1kHz 级实时控制环）。
/// 每个控制周期：
///   1. 下发平台低频指令；
///   2. 读取平台编码器实际位置（含滞后/扰动）；
///   3. 计算平台跟随误差 e = 指令 - 实测；
///   4. 将误差矢量前馈补偿到振镜指令：galvoCmd = 高频分量 + e；
///   5. 激光落点 = 平台实际位置 + 振镜实际偏摆。
/// </summary>
public sealed class LinkageSimulator
{
    private readonly DecomposeResult _plan;
    private readonly StageAxisModel _stageX;
    private readonly StageAxisModel _stageY;
    private readonly GalvoAxisModel _galvoX = new();
    private readonly GalvoAxisModel _galvoY = new();

    public int Index { get; private set; }
    public int Count => _plan.Count;
    public bool Done => Index >= Count;
    public double Dt => _plan.Raw.Dt;

    /// <summary>是否启用平台误差 → 振镜方向补偿</summary>
    public bool CompensationEnabled { get; set; } = true;

    /// <summary>平台指令前瞻采样数（补偿伺服固有滞后，≈ 2ζ/ωn）</summary>
    public int LeadSamples { get; set; }

    // ---- 历史记录（用于绘制轨迹与误差曲线）----
    public double[] SpotX { get; }
    public double[] SpotY { get; }
    public double[] StageActX { get; }
    public double[] StageActY { get; }
    public double[] StageErrX { get; }      // 平台跟随误差
    public double[] StageErrY { get; }
    public double[] SpotError { get; }      // 激光落点 vs 理想轨迹的合成误差

    // ---- 实时状态 ----
    public double CurStageCmdX { get; private set; }
    public double CurStageCmdY { get; private set; }
    public double CurStageActX { get; private set; }
    public double CurStageActY { get; private set; }
    public double CurGalvoX { get; private set; }
    public double CurGalvoY { get; private set; }
    public double CurSpotX { get; private set; }
    public double CurSpotY { get; private set; }
    public double CurStageErr { get; private set; }
    public double CurSpotErr { get; private set; }
    public bool CurLaserOn => Index > 0 && _plan.Raw.LaserOn[Math.Min(Index - 1, Count - 1)];

    // ---- 统计 ----
    public double MaxSpotError { get; private set; }
    private double _sumSqErr; private int _laserOnSamples;
    public double RmsSpotError => _laserOnSamples > 0 ? Math.Sqrt(_sumSqErr / _laserOnSamples) : 0;

    public LinkageSimulator(DecomposeResult plan,
        double stageBandwidthHz, double stageDamping,
        double disturbAmp, double disturbFreq,
        double galvoFov, double galvoTimeConst)
    {
        _plan = plan;
        _stageX = new StageAxisModel(12345)
        { BandwidthHz = stageBandwidthHz, Damping = stageDamping, DisturbanceAmp = disturbAmp, DisturbanceFreq = disturbFreq };
        _stageY = new StageAxisModel(67890)
        { BandwidthHz = stageBandwidthHz, Damping = stageDamping, DisturbanceAmp = disturbAmp, DisturbanceFreq = disturbFreq * 1.31 };
        _galvoX.Fov = galvoFov; _galvoY.Fov = galvoFov;
        _galvoX.TimeConstant = galvoTimeConst; _galvoY.TimeConstant = galvoTimeConst;

        // 伺服等效延迟 ≈ 2ζ/ωn，指令提前下发作为前馈前瞻
        double servoLag = 2 * stageDamping / (2 * Math.PI * stageBandwidthHz);
        LeadSamples = (int)Math.Round(servoLag / plan.Raw.Dt);

        int n = plan.Count;
        SpotX = new double[n]; SpotY = new double[n];
        StageActX = new double[n]; StageActY = new double[n];
        StageErrX = new double[n]; StageErrY = new double[n];
        SpotError = new double[n];
        Reset();
    }

    public void Reset()
    {
        Index = 0;
        MaxSpotError = 0; _sumSqErr = 0; _laserOnSamples = 0;
        if (Count > 0)
        {
            _stageX.Reset(_plan.StageX[0]);
            _stageY.Reset(_plan.StageY[0]);
        }
        _galvoX.Reset(); _galvoY.Reset();
    }

    /// <summary>推进 n 个控制周期</summary>
    public void Step(int n = 1)
    {
        double dt = Dt;
        for (int k = 0; k < n && Index < Count; k++, Index++)
        {
            int i = Index;

            // 1) 平台指令（含前瞻前馈）与实际位置
            double cmdX = _plan.StageX[i];
            double cmdY = _plan.StageY[i];
            int lead = Math.Min(i + LeadSamples, Count - 1);
            double actX = _stageX.Step(_plan.StageX[lead], dt);
            double actY = _stageY.Step(_plan.StageY[lead], dt);

            // 2) 平台跟随误差（实时监控）
            double errX = cmdX - actX;
            double errY = cmdY - actY;

            // 3) 振镜指令 = 高频分量 + 平台误差方向补偿
            double gCmdX = _plan.GalvoX[i] + (CompensationEnabled ? errX : 0);
            double gCmdY = _plan.GalvoY[i] + (CompensationEnabled ? errY : 0);
            double gX = _galvoX.Step(gCmdX, dt);
            double gY = _galvoY.Step(gCmdY, dt);

            // 4) 激光落点 = 平台实际 + 振镜偏摆
            double spotX = actX + gX;
            double spotY = actY + gY;

            double resid = Math.Sqrt(
                (spotX - _plan.Raw.X[i]) * (spotX - _plan.Raw.X[i]) +
                (spotY - _plan.Raw.Y[i]) * (spotY - _plan.Raw.Y[i]));

            // 记录
            SpotX[i] = spotX; SpotY[i] = spotY;
            StageActX[i] = actX; StageActY[i] = actY;
            StageErrX[i] = errX; StageErrY[i] = errY;
            SpotError[i] = resid;

            if (_plan.Raw.LaserOn[i])
            {
                if (resid > MaxSpotError) MaxSpotError = resid;
                _sumSqErr += resid * resid;
                _laserOnSamples++;
            }

            // 实时状态
            CurStageCmdX = cmdX; CurStageCmdY = cmdY;
            CurStageActX = actX; CurStageActY = actY;
            CurGalvoX = gX; CurGalvoY = gY;
            CurSpotX = spotX; CurSpotY = spotY;
            CurStageErr = Math.Sqrt(errX * errX + errY * errY);
            CurSpotErr = resid;
        }
    }
}
