using System;
using System.Collections.Generic;
using System.Linq;
using GalvoStage.Core.Geometry;

namespace GalvoStage.Core.PathPlanning;

/// <summary>等时间间隔采样后的加工轨迹（联动控制的指令基准）</summary>
public sealed class SampledTrajectory
{
    public double SampleRate { get; init; }              // Hz
    public double Dt => 1.0 / SampleRate;
    public double[] X { get; init; } = Array.Empty<double>();
    public double[] Y { get; init; } = Array.Empty<double>();
    public bool[] LaserOn { get; init; } = Array.Empty<bool>();
    public int Count => X.Length;
    public double Duration => Count * Dt;
}

/// <summary>
/// 将 DXF 折线集合转换为按进给速度等时采样的轨迹：
/// 1) 贪心最近邻排序减少空程；2) 空程按快移速度插补(激光关)；3) 轮廓按进给速度插补(激光开)。
/// </summary>
public static class PathSampler
{
    public static SampledTrajectory Sample(IReadOnlyList<PathPolyline> polylines,
        double feedSpeed, double rapidSpeed, double sampleRate)
    {
        var ordered = OrderByNearest(polylines);
        var xs = new List<double>(1 << 16);
        var ys = new List<double>(1 << 16);
        var laser = new List<bool>(1 << 16);
        double dt = 1.0 / sampleRate;

        Vec2 cur = ordered.Count > 0 ? ordered[0].Points[0] : Vec2.Zero;
        double residual = 0;   // 上一段剩余的采样相位距离

        foreach (var pl in ordered)
        {
            var pts = new List<Vec2>(pl.Points);
            if (pl.Closed && pts.Count > 1 && !pts[0].Equals(pts[^1])) pts.Add(pts[0]);

            // 空程：当前位置 -> 轮廓起点
            residual = InterpolateSegment(cur, pts[0], rapidSpeed * dt, residual, xs, ys, laser, false);
            cur = pts[0];

            // 轮廓段
            for (int i = 1; i < pts.Count; i++)
            {
                residual = InterpolateSegment(cur, pts[i], feedSpeed * dt, residual, xs, ys, laser, true);
                cur = pts[i];
            }
        }
        // 末尾补一个终点采样
        if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
        { xs.Add(cur.X); ys.Add(cur.Y); laser.Add(false); }

        return new SampledTrajectory
        {
            SampleRate = sampleRate,
            X = xs.ToArray(),
            Y = ys.ToArray(),
            LaserOn = laser.ToArray()
        };
    }

    /// <summary>沿线段以固定步距采样，返回跨段剩余相位</summary>
    private static double InterpolateSegment(Vec2 a, Vec2 b, double step, double residual,
        List<double> xs, List<double> ys, List<bool> laser, bool laserOn)
    {
        double len = a.DistanceTo(b);
        if (len < 1e-12) return residual;
        double s = step - residual;      // 本段第一个采样点的弧长位置
        while (s <= len)
        {
            double t = s / len;
            xs.Add(a.X + (b.X - a.X) * t);
            ys.Add(a.Y + (b.Y - a.Y) * t);
            laser.Add(laserOn);
            s += step;
        }
        return step - (s - len);         // 新的剩余相位
    }

    /// <summary>贪心最近邻排序（可整体反向折线）</summary>
    private static List<PathPolyline> OrderByNearest(IReadOnlyList<PathPolyline> input)
    {
        var remaining = input.Where(p => p.Points.Count > 1).ToList();
        var ordered = new List<PathPolyline>(remaining.Count);
        Vec2 cur = Vec2.Zero;
        while (remaining.Count > 0)
        {
            int bestIdx = 0; bool reverse = false; double bestDist = double.MaxValue;
            for (int i = 0; i < remaining.Count; i++)
            {
                double dHead = cur.DistanceTo(remaining[i].Points[0]);
                double dTail = cur.DistanceTo(remaining[i].Points[^1]);
                if (dHead < bestDist) { bestDist = dHead; bestIdx = i; reverse = false; }
                if (!remaining[i].Closed && dTail < bestDist) { bestDist = dTail; bestIdx = i; reverse = true; }
            }
            var pick = remaining[bestIdx];
            remaining.RemoveAt(bestIdx);
            if (reverse)
            {
                var rp = new PathPolyline { Closed = pick.Closed, Layer = pick.Layer };
                for (int i = pick.Points.Count - 1; i >= 0; i--) rp.Points.Add(pick.Points[i]);
                pick = rp;
            }
            ordered.Add(pick);
            cur = pick.Closed ? pick.Points[0] : pick.Points[^1];
        }
        return ordered;
    }
}
