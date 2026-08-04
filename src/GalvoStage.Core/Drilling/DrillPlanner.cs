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
    /// <param name="pattern">钻孔点集</param>
    /// <param name="dwellTimeMs">单孔停留时间 (ms)</param>
    /// <param name="galvoFov">振镜半视场 (mm)，用于振镜优先聚类的网格尺寸；&lt;=0 时退化为纯路径最短</param>
    /// <param name="galvoFirst">振镜优先策略：按振镜视场网格聚类，簇内全走振镜，仅簇间才动平台——大幅减少平台跳跃次数</param>
    public static DrillingTrajectory Plan(Geometry.Drilling.DrillingPattern pattern,
        double dwellTimeMs = 50.0, double galvoFov = 5.0, bool galvoFirst = false)
    {
        if (pattern.Holes.Count == 0)
            return new DrillingTrajectory();

        var ordered = galvoFirst
            ? PlanGalvoFirst(pattern.Holes, galvoFov)
            : pattern.Holes.Count > MaxPointsPerZone
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

    private const int MaxPointsPerZone = 5_000;

    /// <summary>
    /// 振镜优先路径规划（大数据场景核心优化）。
    /// 算法：
    ///   1. 以振镜全视场 (2·FOV) 为网格尺寸，将孔聚类到簇；
    ///   2. 按簇质心的莫顿码（Z-order）排序簇访问顺序，保证相邻簇空间相邻；
    ///   3. 簇内按最近邻贪心排序，让振镜在簇内走短路径。
    /// 效果：平台仅在簇间跳跃（K 次，K=簇数），簇内全部由振镜完成。
    /// 百万孔 / ±FOV=5mm / 600×400mm 板 → 约 4800 簇，平台动 4800 次（而非百万次）。
    /// </summary>
    private static List<Geometry.Drilling.DrillingPattern.Hole> PlanGalvoFirst(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, double galvoFov)
    {
        int n = holes.Count;
        if (n <= 1) return new List<Geometry.Drilling.DrillingPattern.Hole>(holes);

        // 1. 全局包围盒
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        foreach (var h in holes)
        {
            if (h.X < minX) minX = h.X;
            if (h.Y < minY) minY = h.Y;
            if (h.X > maxX) maxX = h.X;
            if (h.Y > maxY) maxY = h.Y;
        }

        // 2. 网格尺寸 = 振镜全视场 (2·FOV)；FOV 过小则退化为 1mm 网格
        double cellSize = Math.Max(galvoFov > 0 ? 2 * galvoFov : 1.0, 1e-3);
        int dimX = Math.Max(1, (int)Math.Ceiling((maxX - minX) / cellSize));
        int dimY = Math.Max(1, (int)Math.Ceiling((maxY - minY) / cellSize));
        int totalCells = dimX * dimY;

        // 2.5 密度检查：平均孔数/单元 < 4 时，GF 会把路径打散成低效网格遍历，
        //     回退到 Z-order 或最近邻排序（这些策略在稀疏数据上更紧凑）。
        double density = (double)n / totalCells;
        if (density < 4.0)
        {
            return n > MaxPointsPerZone
                ? OrderByZonal(holes, MaxPointsPerZone)
                : OrderByNearestGrid(holes);
        }

        // 3. 计数排序分桶：O(n) 把孔分配到 (dimX × dimY) 网格
        var cellOf = new int[n];
        var cellCount = new int[totalCells];
        for (int i = 0; i < n; i++)
        {
            int cx = Math.Clamp((int)((holes[i].X - minX) / cellSize), 0, dimX - 1);
            int cy = Math.Clamp((int)((holes[i].Y - minY) / cellSize), 0, dimY - 1);
            int cell = cy * dimX + cx;
            cellOf[i] = cell;
            cellCount[cell]++;
        }

        // 4. 收集非空簇，建立 cellId → clusterId 映射，计算质心
        int K = 0;
        for (int c = 0; c < totalCells; c++)
            if (cellCount[c] > 0) K++;

        var clusterCellId = new int[K];       // 簇对应的 cell id
        var clusterCx = new double[K];        // 簇质心 X
        var clusterCy = new double[K];        // 簇质心 Y
        var clusterMembers = new List<int>[K]; // 簇内孔索引
        var cellToCluster = new int[totalCells]; // cellId → clusterId（-1 表示空）
        Array.Fill(cellToCluster, -1);

        int ki = 0;
        for (int c = 0; c < totalCells; c++)
        {
            if (cellCount[c] == 0) continue;
            cellToCluster[c] = ki;
            clusterCellId[ki] = c;
            clusterMembers[ki] = new List<int>(cellCount[c]);
            ki++;
        }
        for (int i = 0; i < n; i++)
        {
            int ci = cellToCluster[cellOf[i]];
            clusterMembers[ci].Add(i);
            clusterCx[ci] += holes[i].X;
            clusterCy[ci] += holes[i].Y;
        }
        for (int ci = 0; ci < K; ci++)
        {
            int cnt = clusterMembers[ci].Count;
            clusterCx[ci] /= cnt;
            clusterCy[ci] /= cnt;
        }

        // 5. 按簇质心莫顿码排序（O(K log K)），保证相邻簇在空间上相邻
        var clusterOrder = OrderClustersByMorton(clusterCx, clusterCy, dimX, dimY);

        // 6. 簇内按最近邻排序，拼接成最终序列
        var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
        foreach (int ci in clusterOrder)
        {
            var members = clusterMembers[ci];
            // 从簇中心出发，贪心走最近未访问孔
            double px = clusterCx[ci], py = clusterCy[ci];
            var usedInCluster = new bool[members.Count];
            for (int step = 0; step < members.Count; step++)
            {
                int bestIdx = -1;
                double bestD2 = double.MaxValue;
                for (int j = 0; j < members.Count; j++)
                {
                    if (usedInCluster[j]) continue;
                    int idx = members[j];
                    double dx = holes[idx].X - px;
                    double dy = holes[idx].Y - py;
                    double d2 = dx * dx + dy * dy;
                    if (d2 < bestD2) { bestD2 = d2; bestIdx = j; }
                }
                usedInCluster[bestIdx] = true;
                ordered.Add(holes[members[bestIdx]]);
                px = holes[members[bestIdx]].X;
                py = holes[members[bestIdx]].Y;
            }
        }

        return ordered;
    }

    /// <summary>按簇质心的莫顿码排序，返回簇索引的访问顺序</summary>
    private static int[] OrderClustersByMorton(double[] cx, double[] cy, int dimX, int dimY)
    {
        int K = cx.Length;
        int bits = 0;
        int temp = Math.Max(dimX, dimY) - 1;
        while (temp > 0) { bits++; temp >>= 1; }

        var codes = new (ulong code, int idx)[K];
        for (int i = 0; i < K; i++)
        {
            int x = Math.Clamp((int)cx[i], 0, dimX - 1);
            int y = Math.Clamp((int)cy[i], 0, dimY - 1);
            codes[i] = (EncodeMorton64(x, y, bits), i);
        }
        Array.Sort(codes, (a, b) => a.code.CompareTo(b.code));
        var order = new int[K];
        for (int i = 0; i < K; i++) order[i] = codes[i].idx;
        return order;
    }

    /// <summary>64 位莫顿码（支持更大网格）</summary>
    private static ulong EncodeMorton64(int x, int y, int bits)
    {
        ulong result = 0;
        for (int i = 0; i < bits; i++)
        {
            result |= ((ulong)(x & (1 << i)) << (2 * i)) |
                      ((ulong)(y & (1 << i)) << (2 * i + 1));
        }
        return result;
    }

    /// <summary>Z-order 曲线（莫顿码）分区排序（用于超大规模数据）</summary>
    /// <param name="holes">输入孔位列表</param>
    /// <param name="maxPerZone">每个区域的最大孔数阈值（可选参数，当前实现已忽略）</param>
    private static List<Geometry.Drilling.DrillingPattern.Hole> OrderByZonal(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, int maxPerZone = 0)
    {
        int n = holes.Count;
        if (n <= 1) return new List<Geometry.Drilling.DrillingPattern.Hole>(holes);
        
        // 1. 计算全局包围盒
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        foreach (var h in holes)
        {
            if (h.X < minX) minX = h.X;
            if (h.Y < minY) minY = h.Y;
            if (h.X > maxX) maxX = h.X;
            if (h.Y > maxY) maxY = h.Y;
        }
        
        double width = Math.Max(maxX - minX, 1e-9);
        double height = Math.Max(maxY - minY, 1e-9);
        
        // 2. 确定网格分辨率（方形网格）
        int dim = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(n)));
        double cw = width / dim;
        double ch = height / dim;
        
        // 3. 计算莫顿码并分配单元格
        var coded = new MortonCode[n];
        for (int i = 0; i < n; i++)
        {
            int cx = (int)((holes[i].X - minX) / cw);
            int cy = (int)((holes[i].Y - minY) / ch);
            cx = Math.Clamp(cx, 0, dim - 1);
            cy = Math.Clamp(cy, 0, dim - 1);
            
            uint cellIndex = EncodeMortonCode(cx, cy, dim);
            coded[i] = new MortonCode { OriginalIndex = i, Code = cellIndex, CellX = cx, CellY = cy };
        }
        
        // 4. 按莫顿码排序（使用快速排序）
        Array.Sort(coded, (a, b) => a.Code.CompareTo(b.Code));
        
        // 5. 重建有序列表
        var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
        for (int i = 0; i < n; i++)
            ordered.Add(holes[coded[i].OriginalIndex]);
        
        return ordered;
    }
    
    /// <summary>二阶莫顿码（Z-order）编码</summary>
    /// <param name="x">单元格 X 坐标</param>
    /// <param name="y">单元格 Y 坐标</param>
    /// <param name="gridSize">网格尺寸（必须是 2 的幂次）</param>
    /// <returns>32 位莫顿码</returns>
    private static uint EncodeMortonCode(int x, int y, int gridSize)
    {
        // 将 grid size 向上舍入到最接近的 2 的幂次
        int bits = 0;
        int temp = gridSize - 1;
        while (temp > 0)
        {
            bits++;
            temp >>= 1;
        }
        
        uint result = 0;
        for (int i = 0; i < bits; i++)
        {
            result |= ((uint)(x & (1 << i)) << (2 * i)) | 
                      ((uint)(y & (1 << i)) << (2 * i + 1));
        }
        
        return result;
    }
    
    /// <summary>莫顿码数据结构</summary>
    private sealed class MortonCode
    {
        public int OriginalIndex;      // 原始索引
        public uint Code;              // 计算的莫顿码
        public int CellX, CellY;       // 单元格坐标（调试用）
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
