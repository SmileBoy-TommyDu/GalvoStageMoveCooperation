using System;
using System.Collections.Generic;
using GalvoStage.Core.Geometry;

namespace GalvoStage.Core.Drilling;

/// <summary>
/// PCB 钻孔路径规划器
/// 将孔位列表优化为最短加工路径（网格加速最近邻排序）
/// </summary>
public static class DrillPlanner
{
    /// <summary>单个钻孔移动指令</summary>
    public sealed class HoleMove
    {
        public Vec2 Position;
        public bool IsRapid;      // 快移（激光关）
        public bool IsDrilling;   // 钻孔（主轴开）
        public double DwellTimeMs; // 停留时间 (ms)
        public double Diameter;    // 孔径 (mm)，0 表示未知
        public string Layer;       // 来源图层
        
        public override string ToString() => $"({Position.X:F3},{Position.Y:F3})";
    }

    /// <summary>完整钻孔轨迹</summary>
    public sealed class DrillingTrajectory
    {
        public List<HoleMove> Moves { get; } = new();
        public int HoleCount => Moves.Count;
        public double TotalDurationMs => Moves.Count * 50; // 简化估算：每孔 50ms
        
        public override string ToString() => $"{HoleCount:N0} 孔，~{(TotalDurationMs/1000):.0}s";
    }

    /// <summary>生成优化后的钻孔路径</summary>
    public static DrillingTrajectory Plan(Geometry.Drilling.DrillingPattern pattern,
        double dwellTimeMs = 50.0)
    {
        if (pattern.Holes.Count == 0)
            return new DrillingTrajectory();
        
        const int MaxPointsPerZone = 5_000;
        var ordered = pattern.Holes.Count > MaxPointsPerZone 
            ? OrderByZonal(pattern.Holes, MaxPointsPerZone)
            : OrderByNearestGrid(pattern.Holes);
        
        var trajectory = new DrillingTrajectory();
        for (int i = 0; i < ordered.Count; i++)
        {
            var h = ordered[i];
            trajectory.Moves.Add(new HoleMove
            {
                Position = new Vec2(h.X, h.Y),
                IsRapid = i == 0,
                IsDrilling = true,
                DwellTimeMs = dwellTimeMs,
                Diameter = h.Diameter,
                Layer = h.Layer
            });
        }
        
        return trajectory;
    }

    /// <summary>Z-order 分区排序（用于超大数据）</summary>
    private static List<Geometry.Drilling.DrillingPattern.Hole> OrderByZonal(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, int maxPerZone)
    {
        // 计算包围盒
        var bounds = holes[0];
        for (int i = 1; i < holes.Count; i++)
        {
            var h = holes[i];
            if (h.X < bounds.X) bounds.X = h.X;
            if (h.Y < bounds.Y) bounds.Y = h.Y;
            if (h.X > bounds.X) bounds.X = h.X;
            if (h.Y > bounds.Y) bounds.Y = h.Y;
        }
        
        double width = bounds.X - (bounds.X - 0); // 简化：暂不实现 Z-order
        // TODO: 完整 Z-order 曲线分区
        return OrderByNearestGrid(holes); // 降级为网格排序
    }

    /// <summary>网格加速最近邻排序（核心算法）</summary>
    private static List<Geometry.Drilling.DrillingPattern.Hole> OrderByNearestGrid(
        List<Geometry.Drilling.DrillingPattern.Hole> holes)
    {
        int n = holes.Count;
        if (n <= 1) return holes;
        
        // 端点表（每个点就是一个"端点"，闭合性不适用）
        var ex = new List<double>(n);
        var ey = new List<double>(n);
        var epoly = new List<int>(n); // 索引到 holes
        
        for (int i = 0; i < n; i++)
        {
            ex.Add(holes[i].X);
            ey.Add(holes[i].Y);
            epoly.Add(i);
        }
        
        int m = ex.Count;
        
        // 全局包围盒 → 均匀网格
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        for (int k = 0; k < m; k++)
        {
            if (ex[k] < minX) minX = ex[k];
            if (ex[k] > maxX) maxX = ex[k];
            if (ey[k] < minY) minY = ey[k];
            if (ey[k] > maxY) maxY = ey[k];
        }
        
        int dim = Math.Max(1, (int)Math.Sqrt(m));
        double cw = Math.Max((maxX - minX) / dim, 1e-9);
        double ch = Math.Max((maxY - minY) / dim, 1e-9);
        
        // 计数排序建桶
        var starts = new int[dim * dim + 1];
        var cellOf = new int[m];
        for (int k = 0; k < m; k++)
        {
            int cx = Math.Clamp((int)((ex[k] - minX) / cw), 0, dim - 1);
            int cy = Math.Clamp((int)((ey[k] - minY) / ch), 0, dim - 1);
            int cell = cy * dim + cx;
            cellOf[k] = cell;
            starts[cell + 1]++;
        }
        for (int b = 0; b < dim * dim; b++) starts[b + 1] += starts[b];
        var items = new int[m];
        var fill = (int[])starts.Clone();
        for (int k = 0; k < m; k++) items[fill[cellOf[k]]++] = k;
        
        var used = new bool[n];      // 孔级懒删除
        var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
        double px = minX;             // 从最小 x 开始（避免从零原点出发造成第一次大空程）
        double py = minY;
        double minCell = Math.Min(cw, ch);
        
        for (int step = 0; step < n; step++)
        {
            int ccx = Math.Clamp((int)((px - minX) / cw), 0, dim - 1);
            int ccy = Math.Clamp((int)((py - minY) / ch), 0, dim - 1);
            int best = -1;
            double bestD2 = double.MaxValue;
            
            for (int r = 0; r <= 2 * dim; r++)
            {
                if (best >= 0 && r > 0)
                {
                    double ringMin = (r - 1) * minCell;
                    if (ringMin > 0 && ringMin * ringMin > bestD2) break;
                }
                
                int xlo = ccx - r, xhi = ccx + r, ylo = ccy - r, yhi = ccy + r;
                for (int cy = Math.Max(ylo, 0); cy <= Math.Min(yhi, dim - 1); cy++)
                {
                    bool edgeRow = cy == ylo || cy == yhi;
                    for (int cx = Math.Max(xlo, 0); cx <= Math.Min(xhi, dim - 1); cx++)
                    {
                        if (!edgeRow && cx != xlo && cx != xhi) continue;
                        
                        int cell = cy * dim + cx;
                        for (int t = starts[cell]; t < starts[cell + 1]; t++)
                        {
                            int k = items[t];
                            if (used[k]) continue;
                            
                            double dx = ex[k] - px, dy = ey[k] - py;
                            double d2 = dx * dx + dy * dy;
                            if (d2 < bestD2)
                            {
                                bestD2 = d2;
                                best = k;
                            }
                        }
                    }
                }
                
                if (r >= dim && best >= 0) break;
            }
            
            if (best < 0) break;
            
            int pi = epoly[best];
            used[pi] = true;
            var pick = holes[pi];
            ordered.Add(pick);
            px = pick.X;
            py = pick.Y;
        }
        
        return ordered;
    }
}
