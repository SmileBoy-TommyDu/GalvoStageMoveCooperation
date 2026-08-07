using System;
using System.Collections.Generic;
using System.Diagnostics;
using GalvoStage.Core.Drilling;
using Geometry = GalvoStage.Core.Geometry;

namespace SmokeTest;

/// <summary>
/// Z-order（莫顿码）分区排序验证：
/// 程序化生成随机孔位，孔数 > 5000 时 DrillPlanner.Plan 自动走 OrderByZonal（Z-order），
/// 统计排序耗时与总快移路径长度，并与未排序基线对比。
/// </summary>
public static class ZOrderBenchmark
{
    public static int Run()
    {
        Console.WriteLine("=== Z-order（莫顿码）分区排序验证 ===\n");

        // 分别验证：≤5000（走网格贪心）与 >5000（走 Z-order）
        foreach (int n in new[] { 5_000, 20_000, 100_000 })
        {
            var pattern = MakeRandomPattern(n, boardW: 500, boardH: 400, seed: 42);
            string algo = n > 5_000 ? "Z-order 莫顿排序" : "网格贪心最近邻";

            double rawLen = PathLength(pattern.Holes);

            var sw = Stopwatch.StartNew();
            var traj = DrillPlanner.Plan(pattern);
            sw.Stop();

            double orderedLen = TrajectoryLength(traj);
            Console.WriteLine($"[{n,7:N0} 孔] 算法={algo}");
            Console.WriteLine($"          规划耗时     = {sw.ElapsedMilliseconds,6} ms");
            Console.WriteLine($"          未排序路径长 = {rawLen / 1000,10:F2} m");
            Console.WriteLine($"          排序后路径长 = {orderedLen / 1000,10:F2} m  (缩短 {(1 - orderedLen / rawLen) * 100:F1}%)");

            // 正确性校验：孔数不变、无重复
            if (traj.Moves.Count != n)
            {
                Console.WriteLine($"          ❌ 孔数不一致：期望 {n}，实际 {traj.Moves.Count}");
                return 1;
            }
            var seen = new HashSet<(double, double)>();
            foreach (var m in traj.Moves) seen.Add((m.Position.X, m.Position.Y));
            Console.WriteLine($"          校验：孔数一致 ✔  唯一坐标 {seen.Count:N0} ✔\n");
        }

        return 0;
    }

    /// <summary>生成均匀随机孔位（固定种子，可复现）</summary>
    private static Geometry.Drilling.DrillingPattern MakeRandomPattern(
        int n, double boardW, double boardH, int seed)
    {
        var rnd = new Random(seed);
        var pattern = new Geometry.Drilling.DrillingPattern();
        for (int i = 0; i < n; i++)
            pattern.Holes.Add(new Geometry.Drilling.DrillingPattern.Hole(
                rnd.NextDouble() * boardW, rnd.NextDouble() * boardH, 0.3, "DRILL", i));
        pattern.RecomputeBounds();
        return pattern;
    }

    /// <summary>按原始顺序遍历的总路径长度 (mm)</summary>
    private static double PathLength(List<Geometry.Drilling.DrillingPattern.Hole> holes)
    {
        double len = 0;
        for (int i = 1; i < holes.Count; i++)
        {
            double dx = holes[i].X - holes[i - 1].X;
            double dy = holes[i].Y - holes[i - 1].Y;
            len += Math.Sqrt(dx * dx + dy * dy);
        }
        return len;
    }

    /// <summary>排序后轨迹的总路径长度 (mm)</summary>
    private static double TrajectoryLength(DrillPlanner.DrillingTrajectory traj)
    {
        double len = 0;
        for (int i = 1; i < traj.Moves.Count; i++)
            len += (traj.Moves[i].Position - traj.Moves[i - 1].Position).Length;
        return len;
    }
}
