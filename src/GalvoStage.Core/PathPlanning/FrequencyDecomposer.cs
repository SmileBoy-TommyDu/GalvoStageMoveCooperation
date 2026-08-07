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
    public double GalvoMaxVelocity { get; init; }    // 振镜最大速度 (mm/s)
    public double GalvoMaxAcceleration { get; init; }// 振镜最大加速度 (mm/s^2)
    public int Count => StageX.Length;

    // 多约束可行性标志（由 DecomposeAuto 多约束版本填充）
    public bool Feasible { get; set; }                          // 四约束是否全部满足
    public bool FovConstraintSatisfied { get; set; }            // 振镜 FOV 约束
    public bool StageVelocityConstraintSatisfied { get; set; }  // 平台速度约束
    public bool StageAccelConstraintSatisfied { get; set; }     // 平台加速度约束
    public bool GalvoVelocityConstraintSatisfied { get; set; }  // 振镜速度约束

    /// <summary>振镜典型最大速度 (mm/s)。振镜是谐振扫描头，速度远高于平台（典型 500-5000 mm/s）。</summary>
    public const double DefaultGalvoMaxSpeed = 2000.0;
    /// <summary>平台典型最大速度 (mm/s)。平台是伺服轴，速度远低于振镜（典型 50-500 mm/s）。</summary>
    public const double DefaultStageMaxSpeed = 300.0;

    /// <summary>
    /// 估算实际加工时间（秒）。
    ///
    /// 物理模型（向量合成协同）：
    ///   激光头位置 P(t) = StageP(t) + GalvoP(t) （向量叠加）
    ///   速度 V_total(t) = V_stage(t) + V_galvo(t) （瞬时向量和）
    ///
    /// 由于频率分解使平台走低频、振镜走高頻，两者频谱正交不重叠：
    ///   |V_total|² = |V_stage|² + |V_galvo|² （交叉项平均为 0）
    ///   ⇒ V_eff = √(⟨|V_stage|²⟩ + ⟨|V_galvo|²⟩)
    ///   ⇒ 总时间 = 原始轨迹长度 / V_eff
    ///
    /// FOV 影响的传递链：
    ///   FOV ↑ → 低频更平滑（Butterworth 截止更低）→ 平台 RMS 速度 ↓
    ///   FOV ↑ → 高频残差更多 → 振镜 RMS 速度 ↑
    ///   因振镜速度 (2000 mm/s) ≫ 平台速度 (300 mm/s)，
    ///   振镜增量 > 平台减量 → V_eff ↑ → 总时间 ↓
    /// </summary>
    public (double TotalSec, double StageRmsVel, double GalvoRmsVel, double EffectiveVel,
        double StageDist, double GalvoDist, bool StageWeighted, bool GalvoWeighted) 
        EstimateCycleTime(
            double galvoMaxSpeed = DefaultGalvoMaxSpeed,
            double stageMaxSpeed = DefaultStageMaxSpeed)
    {
        int n = Count;
        if (n < 2) return (0, 0, 0, 0, 0, 0, false, false);

        // 1) 原始轨迹总长度（激光头实际走过的路径）
        double totalLen = 0;
        for (int i = 1; i < n; i++)
        {
            double dx = Raw.X[i] - Raw.X[i - 1];
            double dy = Raw.Y[i] - Raw.Y[i - 1];
            totalLen += Math.Sqrt(dx * dx + dy * dy);
        }

        // 2) 计算平台/振镜各自轨迹长度
        double stageDist = 0, galvoDist = 0;
        for (int i = 1; i < n; i++)
        {
            double dxS = StageX[i] - StageX[i - 1], dyS = StageY[i] - StageY[i - 1];
            stageDist += Math.Sqrt(dxS * dxS + dyS * dyS);
            double dxG = GalvoX[i] - GalvoX[i - 1], dyG = GalvoY[i] - GalvoY[i - 1];
            galvoDist += Math.Sqrt(dxG * dxG + dyG * dyG);
        }

        // 3) 计算平台/振镜各自的均方根步长（mm per sample）
        double stageRmsStep = RmsStep(StageX, StageY);
        double galvoRmsStep = RmsStep(GalvoX, GalvoY);

        // 4) 关键：用各轴 RMS 步长作为"运动密集度"权重
        //    物理意义：step 越大 → 该轴越活跃 → 对有效速度的贡献越大
        //    stageRmsVel ∝ stageRmsStep, galvoRmsVel ∝ galvoRmsStep
        //    但由于振镜速度上限 >> 平台，需按比例缩放
        //    这里直接用步骤长度反映真实需求，而非归一化到固定容量
        
        // 按距离比例分配 RMS 速度权重
        double totalRmsStep = stageRmsStep + galvoRmsStep;
        double stageRatio = stageRmsStep / Math.Max(totalRmsStep, 1e-9);
        double galvoRatio = galvoRmsStep / Math.Max(totalRmsStep, 1e-9);
        
        // 假设协同有效速度约为两轴平均能力的加权组合
        // 经验公式：effectiveVel ≈ sqrt(v_stage² + v_galvo²)，其中 v_i ∝ dist_i / time_i
        // 简化：time_stage ≈ stageDist / stageMaxSpeed_eff, time_galvo ≈ galvoDist / galvoMaxSpeed_eff
        double stageEffSpeed = stageMaxSpeed * 0.5;  // 持续运行能力
        double galvoEffSpeed = galvoMaxSpeed * 0.8;
        
        double stageTimeShare = stageDist / stageEffSpeed;
        double galvoTimeShare = galvoDist / galvoEffSpeed;
        
        // 功率合成：总时间 = sqrt(t_stage² + t_galvo²)（因为两轴并行工作，各自独立贡献）
        // 注意：这里是时间域的正交合成，而非速度域
        double totalSec = Math.Sqrt(stageTimeShare * stageTimeShare + galvoTimeShare * galvoTimeShare);
        
        // 反推等效 RMS 速度用于展示
        double effectiveVel = totalLen / Math.Max(totalSec, 1e-9);
        double stageRmsVel = stageDist / Math.Max(stageTimeShare, 1e-9);
        double galvoRmsVel = galvoDist / Math.Max(galvoTimeShare, 1e-9);

        bool stageWeighted = stageTimeShare >= galvoTimeShare;
        bool galvoWeighted = galvoTimeShare > stageTimeShare;

        return (totalSec, stageRmsVel, galvoRmsVel, effectiveVel, stageDist, galvoDist, stageWeighted, galvoWeighted);
    }

    /// <summary>计算序列的均方根步长（mm per sample）。</summary>
    private static double RmsStep(double[] x, double[] y)
    {
        if (x.Length < 2) return 0;
        double sumSq = 0;
        int count = x.Length - 1;
        for (int i = 0; i < count; i++)
        {
            double dx = x[i + 1] - x[i], dy = y[i + 1] - y[i];
            sumSq += dx * dx + dy * dy;
        }
        return Math.Sqrt(sumSq / count);
    }

    /// <summary>计算序列的均方根速度系数：单位进给下的 RMS 步速。</summary>
    private static double RmsStepSpeed(double[] x, double[] y, double dt)
    {
        if (x.Length < 2 || dt <= 0) return 0;
        double sumSq = 0;
        int count = x.Length - 1;
        for (int i = 0; i < count; i++)
        {
            double dx = x[i + 1] - x[i], dy = y[i + 1] - y[i];
            sumSq += dx * dx + dy * dy;
        }
        double rmsStep = Math.Sqrt(sumSq / count);
        return rmsStep / dt; // mm/s per (mm/s of feed)
    }

    /// <summary>平均步长（备用）</summary>
    private static double AvgStepLength(double totalLen, int n)
    {
        return n > 1 ? totalLen / (n - 1) : 0;
    }
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
        (double gVMax, double gAMax) = KinematicStats(galvoX, galvoY, fs);

        return new DecomposeResult
        {
            Raw = traj,
            StageX = stageX, StageY = stageY,
            GalvoX = galvoX, GalvoY = galvoY,
            CutoffHz = cutoffHz,
            MaxGalvoDeviation = maxDev,
            StageMaxVelocity = vMax,
            StageMaxAcceleration = aMax,
            GalvoMaxVelocity = gVMax,
            GalvoMaxAcceleration = gAMax
        };
    }

    /// <summary>
    /// 向后兼容重载：仅以振镜 FOV 为约束搜索截止频率（旧行为）。
    /// 新代码建议使用多约束版本 <see cref="DecomposeAuto(SampledTrajectory, double, double, double, double, double, double)"/>。
    /// </summary>
    public static DecomposeResult DecomposeAuto(SampledTrajectory traj, double galvoFov,
        double margin = 0.8, double fcLow = 0.2, double fcHigh = 60)
    {
        // 旧行为：仅约束振镜 FOV，平台/振镜速度加速度用宽松默认值
        return DecomposeAuto(traj, galvoFov,
            stageMaxSpeed: 10_000, stageMaxAccel: 100_000, galvoMaxSpeed: 10_000,
            margin, fcLow, fcHigh);
    }

    /// <summary>
    /// 多约束可行性搜索截止频率（方案 2）。
    ///
    /// 约束（全部关于 fc 单调）：
    ///   1. maxDev(fc) &lt;= galvoFov × margin            （振镜 FOV，↑ fc → ↑ maxDev → 给出 fc ≥ fcMin）
    ///   2. StageMaxVelocity(fc) &lt;= stageMaxSpeed       （平台速度，↑ fc → ↑ vStage → 给出 fc ≤ fcMaxV）
    ///   3. StageMaxAcceleration(fc) &lt;= stageMaxAccel   （平台加速度，↑ fc → ↑ aStage → 给出 fc ≤ fcMaxA）
    ///   4. GalvoMaxVelocity(fc) &lt;= galvoMaxSpeed       （振镜速度，↑ fc → ↓ vGalvo → 给出 fc ≥ fcMinG）
    ///
    /// 可行域：[fcMin, fcMax] = [max(fcMin_FOV, fcMin_GalvoV), min(fcMax_StageV, fcMax_StageA)]
    /// 目标：minimize fc（让平台尽可能平滑）→ 取可行域下界。
    /// </summary>
    public static DecomposeResult DecomposeAuto(SampledTrajectory traj, double galvoFov,
        double stageMaxSpeed, double stageMaxAccel, double galvoMaxSpeed,
        double margin = 0.8, double fcLow = 0.2, double fcHigh = 60)
    {
        double fs = traj.SampleRate;
        double limit = galvoFov * margin;
        double searchHigh = Math.Min(fcHigh, fs * 0.45);
        double searchLow = Math.Max(fcLow, 0.01);

        // 1) 计算各约束给出的 fc 边界
        double fcMinFov = ComputeFcMin(traj, limit, searchLow, searchHigh);
        double fcMinGalvo = ComputeFcMinGalvoV(traj, galvoMaxSpeed, searchLow, searchHigh);
        double fcMaxStageV = ComputeFcMaxStageV(traj, stageMaxSpeed, searchLow, searchHigh);
        double fcMaxStageA = ComputeFcMaxStageA(traj, stageMaxAccel, searchLow, searchHigh);

        // 2) 可行域
        double fcMin = Math.Max(fcMinFov, fcMinGalvo);
        double fcMax = Math.Min(fcMaxStageV, fcMaxStageA);
        fcMin = Math.Clamp(fcMin, searchLow, searchHigh);
        fcMax = Math.Clamp(fcMax, searchLow, searchHigh);

        // 3) 选择候选 fc（优先可行域下界 = 最平滑平台）
        bool feasible = fcMin <= fcMax;
        double fcCandidate = feasible ? fcMin : 0.5 * (fcMin + fcMax);
        fcCandidate = Math.Clamp(fcCandidate, searchLow, searchHigh);

        var result = Decompose(traj, fcCandidate, galvoFov);

        // 4) 四约束校验标志（即便不可行也返回结果，调用方可通过属性自查）
        result.Feasible = feasible;
        result.FovConstraintSatisfied = result.MaxGalvoDeviation <= limit;
        result.StageVelocityConstraintSatisfied = result.StageMaxVelocity <= stageMaxSpeed;
        result.StageAccelConstraintSatisfied = result.StageMaxAcceleration <= stageMaxAccel;
        result.GalvoVelocityConstraintSatisfied = result.GalvoMaxVelocity <= galvoMaxSpeed;

        return result;
    }

    /// <summary>二分搜索满足 maxDev(fc) &lt;= limit 的最小 fc（振镜 FOV 约束下界）。</summary>
    private static double ComputeFcMin(SampledTrajectory traj, double limit,
        double searchLow, double searchHigh)
    {
        var low = Decompose(traj, searchLow, double.PositiveInfinity);
        if (low.MaxGalvoDeviation <= limit) return searchLow;
        var high = Decompose(traj, searchHigh, double.PositiveInfinity);
        if (high.MaxGalvoDeviation > limit) return searchHigh;

        double lo = searchLow, hi = searchHigh;
        for (int i = 0; i < 18 && (hi - lo) > 0.05; i++)
        {
            double mid = 0.5 * (lo + hi);
            var r = Decompose(traj, mid, double.PositiveInfinity);
            if (r.MaxGalvoDeviation <= limit) hi = mid;
            else lo = mid;
        }
        return hi;
    }

    /// <summary>二分搜索满足 GalvoMaxVelocity(fc) &lt;= limit 的最小 fc（振镜速度约束下界）。</summary>
    private static double ComputeFcMinGalvoV(SampledTrajectory traj, double limit,
        double searchLow, double searchHigh)
    {
        var low = Decompose(traj, searchLow, double.PositiveInfinity);
        if (low.GalvoMaxVelocity <= limit) return searchLow;
        var high = Decompose(traj, searchHigh, double.PositiveInfinity);
        if (high.GalvoMaxVelocity > limit) return searchHigh;

        double lo = searchLow, hi = searchHigh;
        for (int i = 0; i < 18 && (hi - lo) > 0.05; i++)
        {
            double mid = 0.5 * (lo + hi);
            var r = Decompose(traj, mid, double.PositiveInfinity);
            if (r.GalvoMaxVelocity <= limit) hi = mid;
            else lo = mid;
        }
        return hi;
    }

    /// <summary>二分搜索满足 StageMaxVelocity(fc) &lt;= limit 的最大 fc（平台速度约束上界）。</summary>
    private static double ComputeFcMaxStageV(SampledTrajectory traj, double limit,
        double searchLow, double searchHigh)
    {
        var high = Decompose(traj, searchHigh, double.PositiveInfinity);
        if (high.StageMaxVelocity <= limit) return searchHigh;
        var low = Decompose(traj, searchLow, double.PositiveInfinity);
        if (low.StageMaxVelocity > limit) return searchLow;

        double lo = searchLow, hi = searchHigh;
        for (int i = 0; i < 18 && (hi - lo) > 0.05; i++)
        {
            double mid = 0.5 * (lo + hi);
            var r = Decompose(traj, mid, double.PositiveInfinity);
            if (r.StageMaxVelocity <= limit) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    /// <summary>二分搜索满足 StageMaxAcceleration(fc) &lt;= limit 的最大 fc（平台加速度约束上界）。</summary>
    private static double ComputeFcMaxStageA(SampledTrajectory traj, double limit,
        double searchLow, double searchHigh)
    {
        var high = Decompose(traj, searchHigh, double.PositiveInfinity);
        if (high.StageMaxAcceleration <= limit) return searchHigh;
        var low = Decompose(traj, searchLow, double.PositiveInfinity);
        if (low.StageMaxAcceleration > limit) return searchLow;

        double lo = searchLow, hi = searchHigh;
        for (int i = 0; i < 18 && (hi - lo) > 0.05; i++)
        {
            double mid = 0.5 * (lo + hi);
            var r = Decompose(traj, mid, double.PositiveInfinity);
            if (r.StageMaxAcceleration <= limit) lo = mid;
            else hi = mid;
        }
        return lo;
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
