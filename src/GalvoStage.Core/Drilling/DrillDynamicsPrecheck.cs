using System;
using System.Collections.Generic;
using System.Linq;

namespace GalvoStage.Core.Drilling;

/// <summary>
/// 逐孔环切动力学预检结果（对齐 docs/06 §7.4）。
/// 携带环切频率 f=v/(2πr) 与向心加速度 a=v²/r 的统计量，
/// 用于判定“哪些孔在当前进给下超出平台跟随能力”，并给出建议进给。
/// </summary>
public sealed class DrillDynamicsReport
{
    /// <summary>越界孔径种类数（环切频率超过平台可稳定跟随上限）</summary>
    public int OffenderKindCount { get; init; }
    /// <summary>越界孔总数</summary>
    public int OffenderHoleCount { get; init; }
    /// <summary>平台可稳定跟随的环切频率上限 (Hz) = min(带宽·margin, fs·0.45)</summary>
    public double FrequencyCapHz { get; init; }
    /// <summary>全部受检孔中的最大环切频率 f=v/(2πr) (Hz)</summary>
    public double MaxRingcutFrequencyHz { get; init; }
    /// <summary>全部受检孔中的最大向心加速度 a=v²/r (mm/s²)</summary>
    public double MaxCentripetalAccel { get; init; }
    /// <summary>是否全部可行（无越界孔）</summary>
    public bool AllFeasible => OffenderKindCount == 0;
    /// <summary>按孔径聚合的越界明细（孔数降序前若干）</summary>
    public IReadOnlyList<Offender> Offenders { get; init; } = Array.Empty<Offender>();

    /// <summary>单一越界孔径的动力学明细</summary>
    public readonly record struct Offender(
        double Diameter,       // 孔径 (mm)
        int Count,             // 该孔径孔数
        double FrequencyHz,    // 环切频率 f=v/(2πr)
        double AccelMmPerS2,   // 向心加速度 a=v²/r
        double SuggestedFeed); // 使 f=FrequencyCap 的最大进给 (mm/s)
}

/// <summary>
/// 逐孔动力学预检算法。
/// 判据：仅半径 &gt; 半视场（galvoFovHalf）的孔才可能失败——视场内的孔振镜可独立覆盖，恒可行；
/// 大孔由平台承载大圆，仅当环切频率 f 超过平台可稳定跟随的带宽（bandwidth·margin）时，
/// 高频残差会落到振镜视场外 → 越界。视场本身对大孔非约束。
/// </summary>
public static class DrillDynamicsPrecheck
{
    /// <summary>带宽余量：只信任伺服带宽的 80% 作为可稳定跟随的环切频率上限。</summary>
    public const double BandwidthMargin = 0.8;

    /// <param name="moves">全量孔（非仅仿真预览子集）</param>
    /// <param name="feedSpeed">环切进给速度 v (mm/s)</param>
    /// <param name="stageBandwidthHz">平台伺服带宽 (Hz)</param>
    /// <param name="sampleRateHz">采样率 fs (Hz)，限制频率分解可搜索上限</param>
    /// <param name="galvoFovHalf">振镜半视场 (±mm)</param>
    /// <param name="topN">明细最多列出的孔径种类数（按孔径降序）</param>
    public static DrillDynamicsReport Evaluate(
        IReadOnlyList<DrillPlanner.HoleMove> moves,
        double feedSpeed, double stageBandwidthHz, double sampleRateHz,
        double galvoFovHalf, int topN = 5)
    {
        double v = feedSpeed;
        double fCap = Math.Min(stageBandwidthHz * BandwidthMargin, sampleRateHz * 0.45);
        if (moves == null || moves.Count == 0 || v <= 0 || fCap <= 0)
            return new DrillDynamicsReport { FrequencyCapHz = fCap };

        var offenders = new Dictionary<double, int>();
        double maxF = 0, maxA = 0;
        int offenderHoles = 0;
        foreach (var m in moves)
        {
            double r = m.Diameter * 0.5;
            if (r <= galvoFovHalf) continue;              // 视场内：振镜独立覆盖，恒可行
            double f = v / (2 * Math.PI * r);              // 环切频率
            double a = v * v / r;                          // 向心加速度
            if (f > maxF) maxF = f;
            if (a > maxA) maxA = a;
            if (f > fCap)                                  // 平台带宽跟不上 → 残差超视场
            {
                offenders[m.Diameter] = offenders.TryGetValue(m.Diameter, out int c) ? c + 1 : 1;
                offenderHoles++;
            }
        }

        var detail = offenders
            .OrderByDescending(e => e.Key)
            .Take(topN)
            .Select(kv =>
            {
                double r = kv.Key * 0.5;
                return new DrillDynamicsReport.Offender(
                    Diameter: kv.Key,
                    Count: kv.Value,
                    FrequencyHz: v / (2 * Math.PI * r),
                    AccelMmPerS2: v * v / r,
                    SuggestedFeed: 2 * Math.PI * r * fCap);
            })
            .ToList();

        return new DrillDynamicsReport
        {
            OffenderKindCount = offenders.Count,
            OffenderHoleCount = offenderHoles,
            FrequencyCapHz = fCap,
            MaxRingcutFrequencyHz = maxF,
            MaxCentripetalAccel = maxA,
            Offenders = detail
        };
    }
}
