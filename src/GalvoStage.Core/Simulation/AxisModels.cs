using System;

namespace GalvoStage.Core.Simulation;

/// <summary>
/// XY 平台单轴伺服模型（二阶欠阻尼系统 + 扰动），模拟真实平台的跟随滞后与振动。
/// x'' = wn^2*(u - x) - 2*zeta*wn*x' ，另叠加正弦扰动与随机噪声。
/// </summary>
public sealed class StageAxisModel
{
    private readonly Random _rng;
    private double _pos, _vel, _time;
    private readonly double _distPhase;

    /// <summary>伺服带宽 (Hz)</summary>
    public double BandwidthHz { get; set; } = 15;
    /// <summary>阻尼比</summary>
    public double Damping { get; set; } = 0.85;
    /// <summary>最大速度 (mm/s)，0 表示不限</summary>
    public double MaxVelocity { get; set; } = 500;
    /// <summary>正弦扰动幅值 (mm)，模拟丝杠周期误差/导轨振动</summary>
    public double DisturbanceAmp { get; set; } = 0.02;
    /// <summary>正弦扰动频率 (Hz)</summary>
    public double DisturbanceFreq { get; set; } = 7;
    /// <summary>随机噪声幅值 (mm)</summary>
    public double NoiseAmp { get; set; } = 0.002;

    public double Position { get; private set; }

    public StageAxisModel(int seed)
    {
        _rng = new Random(seed);
        _distPhase = _rng.NextDouble() * Math.PI * 2;
    }

    public void Reset(double pos)
    {
        _pos = pos; _vel = 0; _time = 0;
        Position = pos;
    }

    /// <summary>推进一个采样周期，返回带扰动的“编码器实测位置”</summary>
    public double Step(double cmd, double dt)
    {
        double wn = 2 * Math.PI * BandwidthHz;
        // 细分积分保证数值稳定
        int sub = Math.Max(1, (int)Math.Ceiling(dt * wn * 4));
        double h = dt / sub;
        for (int i = 0; i < sub; i++)
        {
            double acc = wn * wn * (cmd - _pos) - 2 * Damping * wn * _vel;
            _vel += acc * h;
            if (MaxVelocity > 0) _vel = Math.Clamp(_vel, -MaxVelocity, MaxVelocity);
            _pos += _vel * h;
        }
        _time += dt;

        double disturb = DisturbanceAmp * Math.Sin(2 * Math.PI * DisturbanceFreq * _time + _distPhase)
                       + NoiseAmp * (_rng.NextDouble() * 2 - 1);
        Position = _pos + disturb;
        return Position;
    }
}

/// <summary>
/// 振镜单轴模型（一阶惯性环节，时间常数亚毫秒级），带视场限幅。
/// </summary>
public sealed class GalvoAxisModel
{
    private double _pos;

    /// <summary>时间常数 (s)，典型 0.2~0.5ms</summary>
    public double TimeConstant { get; set; } = 0.0003;
    /// <summary>半视场 (±mm)</summary>
    public double Fov { get; set; } = 5;

    public double Position => _pos;

    public void Reset() => _pos = 0;

    public double Step(double cmd, double dt)
    {
        cmd = Math.Clamp(cmd, -Fov, Fov);
        double alpha = 1 - Math.Exp(-dt / TimeConstant);
        _pos += (cmd - _pos) * alpha;
        return _pos;
    }
}
