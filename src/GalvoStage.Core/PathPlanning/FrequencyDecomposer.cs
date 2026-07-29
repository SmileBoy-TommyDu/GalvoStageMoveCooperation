using System;

namespace GalvoStage.Core.PathPlanning;

/// <summary>频率分解结果：平台走低频大行程，振镜走高频小行程</summary>
public sealed class DecomposeResult
{
    public SampledTrajectory Raw { get; init; } = null!;
    public double[] StageX { get; init; } = Array.Empty<double>();   // 低频分量（平台指令）
    public double[] StageY { get; init; } = Array.Empty<double>();
    public double[] GalvoX { get; init; } = Array.Empty<double>();   // 高频残差（振镜指令，相对平台）
    public double[] GalvoY { get; init; } = Array.Empty<double>();

    public double CutoffHz { get; init; }
    public double MaxGalvoDeviation { get; init; }   // 振镜最大偏摆 (mm)
    public double StageMaxVelocity { get; init; }    // mm/s
    public double StageMaxAcceleration { get; init; }// mm/s^2
    public int Count => StageX.Length;
}

/// <summary>
/// 基于频率的路径分解算法：
/// 对等时采样轨迹做零相位二阶 Butterworth 低通滤波（前向+反向），
/// 低频分量交给 XY 平台（大行程、低带宽），残差高频分量交给振镜（小视场、高带宽）。
/// 截止频率可自动搜索：找到能使振镜偏摆落入视场的最低截止频率，
/// 从而最小化平台的速度/加速度需求。
/// </summary>
public static class FrequencyDecomposer
{
    public static DecomposeResult Decompose(SampledTrajectory traj, double cutoffHz, double galvoFov)
    {
        double fs = traj.SampleRate;
        cutoffHz = Math.Clamp(cutoffHz, 0.1, fs * 0.45);

        double[] stageX = FiltFilt(traj.X, cutoffHz, fs);
        double[] stageY = FiltFilt(traj.Y, cutoffHz, fs);

        int n = traj.Count;
        var galvoX = new double[n];
        var galvoY = new double[n];
        double maxDev = 0;
        for (int i = 0; i < n; i++)
        {
            galvoX[i] = traj.X[i] - stageX[i];
            galvoY[i] = traj.Y[i] - stageY[i];
            double dev = Math.Max(Math.Abs(galvoX[i]), Math.Abs(galvoY[i]));
            if (dev > maxDev) maxDev = dev;
        }

        (double vMax, double aMax) = KinematicStats(stageX, stageY, fs);

        return new DecomposeResult
        {
            Raw = traj,
            StageX = stageX, StageY = stageY,
            GalvoX = galvoX, GalvoY = galvoY,
            CutoffHz = cutoffHz,
            MaxGalvoDeviation = maxDev,
            StageMaxVelocity = vMax,
            StageMaxAcceleration = aMax
        };
    }

    /// <summary>
    /// 自动搜索截止频率：二分查找满足 maxDev &lt;= fov * margin 的最低截止频率。
    /// 截止频率越低 → 平台越平滑（加速度小），但振镜残差越大。
    /// </summary>
    public static DecomposeResult DecomposeAuto(SampledTrajectory traj, double galvoFov,
        double margin = 0.8, double fcLow = 0.2, double fcHigh = 60)
    {
        double limit = galvoFov * margin;
        fcHigh = Math.Min(fcHigh, traj.SampleRate * 0.45);

        var high = Decompose(traj, fcHigh, galvoFov);
        if (high.MaxGalvoDeviation > limit) return high;   // 上限仍超视场，返回最优可行

        var low = Decompose(traj, fcLow, galvoFov);
        if (low.MaxGalvoDeviation <= limit) return low;

        DecomposeResult best = high;
        for (int iter = 0; iter < 18 && (fcHigh - fcLow) > 0.05; iter++)
        {
            double mid = 0.5 * (fcLow + fcHigh);
            var r = Decompose(traj, mid, galvoFov);
            if (r.MaxGalvoDeviation <= limit) { best = r; fcHigh = mid; }
            else fcLow = mid;
        }
        return best;
    }

    // ---------------- 二阶 Butterworth 低通 + 零相位滤波 ----------------

    private static double[] FiltFilt(double[] src, double fc, double fs)
    {
        int n = src.Length;
        if (n < 8) return (double[])src.Clone();

        // 边界填充：常值延拓，长度取 2 个截止周期
        int pad = Math.Min(n - 1, (int)(2 * fs / fc));
        var ext = new double[n + 2 * pad];
        for (int i = 0; i < pad; i++) ext[i] = src[0];
        Array.Copy(src, 0, ext, pad, n);
        for (int i = 0; i < pad; i++) ext[pad + n + i] = src[n - 1];

        var (b0, b1, b2, a1, a2) = ButterLp2(fc, fs);
        Filter(ext, b0, b1, b2, a1, a2);
        Array.Reverse(ext);
        Filter(ext, b0, b1, b2, a1, a2);
        Array.Reverse(ext);

        var dst = new double[n];
        Array.Copy(ext, pad, dst, 0, n);
        return dst;
    }

    private static (double b0, double b1, double b2, double a1, double a2) ButterLp2(double fc, double fs)
    {
        double c = 1.0 / Math.Tan(Math.PI * fc / fs);
        double sqrt2 = Math.Sqrt(2);
        double d = 1 + sqrt2 * c + c * c;
        double b0 = 1 / d;
        double b1 = 2 * b0;
        double b2 = b0;
        double a1 = 2 * (1 - c * c) / d;
        double a2 = (1 - sqrt2 * c + c * c) / d;
        return (b0, b1, b2, a1, a2);
    }

    private static void Filter(double[] x, double b0, double b1, double b2, double a1, double a2)
    {
        double x1 = x[0], x2 = x[0];
        double y1 = x[0], y2 = x[0];   // 稳态初始化，避免起始瞬态
        for (int i = 0; i < x.Length; i++)
        {
            double xi = x[i];
            double yi = b0 * xi + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1; x1 = xi;
            y2 = y1; y1 = yi;
            x[i] = yi;
        }
    }

    private static (double vMax, double aMax) KinematicStats(double[] x, double[] y, double fs)
    {
        int n = x.Length;
        if (n < 3) return (0, 0);
        double vMax = 0, aMax = 0;
        double pvx = 0, pvy = 0;
        for (int i = 1; i < n; i++)
        {
            double vx = (x[i] - x[i - 1]) * fs;
            double vy = (y[i] - y[i - 1]) * fs;
            double v = Math.Sqrt(vx * vx + vy * vy);
            if (v > vMax) vMax = v;
            if (i > 1)
            {
                double ax = (vx - pvx) * fs;
                double ay = (vy - pvy) * fs;
                double a = Math.Sqrt(ax * ax + ay * ay);
                if (a > aMax) aMax = a;
            }
            pvx = vx; pvy = vy;
        }
        return (vMax, aMax);
    }
}
