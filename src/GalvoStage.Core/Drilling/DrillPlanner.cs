using System;
using System.Collections.Generic;
using GalvoStage.Core.Geometry;
using GalvoStage.Core.Geometry.Drilling;

namespace GalvoStage.Core.Drilling;

/// <summary>钻孔加工策略（影响孔位访问顺序）</summary>
public enum DrillingStrategy
{
    /// <summary>加工时间最短：纯空间分区 + 分区内 TSP，忽略孔径分组（默认）。</summary>
    TimeOptimal,
    /// <summary>工艺效果优先：按孔径分组，同一种孔径全幅面一次加工完，再加工下一种孔径（组内仍按分区+TSP）。</summary>
    QualityOptimal
}

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
        
        /// <summary>工艺参数（按孔径分档配置）</summary>
        public TrepanParams ProcessParams { get; set; }
        
        public override string ToString() => $"({Position.X:F3},{Position.Y:F3})";
    }

    /// <summary>完整钻孔轨迹</summary>
    public sealed class DrillingTrajectory
    {
        public List<HoleMove> Moves { get; } = new();
        public int HoleCount => Moves.Count;
        public double TotalDurationMs => Moves.Count * 50; // 简化估算：每孔 50ms
        
        /// <summary>采样后的轨迹（用于激光控制）</summary>
        public Core.PathPlanning.SampledTrajectory? SampledTrajectory { get; set; }
        
        public override string ToString() => $"{HoleCount:N0} 孔，~{(TotalDurationMs/1000):.0}s";
    }

    /// <summary>生成优化后的钻孔路径</summary>
    /// <param name="pattern">钻孔点集</param>
    /// <param name="dwellTimeMs">单孔停留时间 (ms)</param>
    /// <param name="galvoFov">振镜半视场 (mm)，用于振镜优先聚类的网格尺寸；&lt;=0 时退化为纯路径最短</param>
    /// <param name="galvoFirst">振镜优先策略：按振镜视场网格聚类，簇内全走振镜，仅簇间才动平台——大幅减少平台跳跃次数</param>
    /// <param name="jumpSpeedPlatform">平台空移速度 (mm/s)，用于采样</param>
    /// <param name="jumpSpeedGalvo">振镜空移速度 (mm/s)，用于采样</param>
    /// <param name="sampleRate">采样频率 (Hz)</param>
    /// <param name="strategy">加工策略：TimeOptimal（时间最短，默认）或 QualityOptimal（工艺优先，按孔径分组）</param>
    public static DrillingTrajectory Plan(Geometry.Drilling.DrillingPattern pattern,
        double dwellTimeMs = 50.0, double galvoFov = 5.0, bool galvoFirst = false,
        double jumpSpeedPlatform = 500.0, double jumpSpeedGalvo = 2000.0, double sampleRate = 1000.0,
        DrillingStrategy strategy = DrillingStrategy.TimeOptimal)
    {
        if (pattern.Holes.Count == 0)
            return new DrillingTrajectory();

        var ordered = strategy == DrillingStrategy.QualityOptimal
            ? OrderByDiameterGroups(pattern.Holes, galvoFov, galvoFirst)
            : OrderHoles(pattern.Holes, galvoFov, galvoFirst);

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
                Layer = h.Layer,
                ProcessParams = h.ProcessParams
            });
        }
        
        // 生成采样轨迹（用于激光控制）
        trajectory.SampledTrajectory = SampleDrillingTrajectory(
            trajectory.Moves, jumpSpeedPlatform, jumpSpeedGalvo, sampleRate, dwellTimeMs);

        return trajectory;
    }

    private const int MaxPointsPerZone = 5_000;

    /// <summary>根据数据规模与振镜优先开关选择排序策略（分区 + 分区内 TSP）。</summary>
    private static List<Geometry.Drilling.DrillingPattern.Hole> OrderHoles(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, double galvoFov, bool galvoFirst)
    {
        return galvoFirst
            ? PlanGalvoFirst(holes, galvoFov)
            : holes.Count > MaxPointsPerZone
                ? OrderByZonal(holes, MaxPointsPerZone)
                : OrderByNearestGrid(holes);
    }

    /// <summary>
    /// 工艺优先：按孔径分组，同一种孔径全幅面一次加工完，再加工下一种孔径。
    /// 孔径按升序加工（量化到 0.001mm 容忍浮点误差）；每个孔径组内部仍按分区+TSP 优化。
    /// </summary>
    private static List<Geometry.Drilling.DrillingPattern.Hole> OrderByDiameterGroups(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, double galvoFov, bool galvoFirst)
    {
        if (holes.Count <= 1) return new List<Geometry.Drilling.DrillingPattern.Hole>(holes);

        var groups = new SortedDictionary<double, List<Geometry.Drilling.DrillingPattern.Hole>>();
        foreach (var h in holes)
        {
            double key = Math.Round(h.Diameter, 3);
            if (!groups.TryGetValue(key, out var list))
            {
                list = new List<Geometry.Drilling.DrillingPattern.Hole>();
                groups[key] = list;
            }
            list.Add(h);
        }

        // 单一孔径：等价于 TimeOptimal，直接走分区+TSP
        if (groups.Count <= 1) return OrderHoles(holes, galvoFov, galvoFirst);

        var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(holes.Count);
        foreach (var kv in groups)  // SortedDictionary → 孔径升序
            ordered.AddRange(OrderHoles(kv.Value, galvoFov, galvoFirst));
        return ordered;
    }

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
        var cellOf = new int[n];  // 记录“每个孔属于哪个格子”的编号
        var cellCount = new int[totalCells];  // 记录“每个格里有多少个孔”
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

        // 6. 簇内先最近邻贪心生成初始路径，再用 2-opt 局部优化（TSP），拼接成最终序列
        var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
        foreach (int ci in clusterOrder)
        {
            var members = clusterMembers[ci];
            // 从簇中心出发，贪心走最近未访问孔，得到初始开路径
            var tour = NearestNeighborInCluster(holes, members, clusterCx[ci], clusterCy[ci]);
            // 簇内孔数不超阀值时用 2-opt 优化（簇受 FOV 约束，通常规模很小）
            if (tour.Count <= TwoOptMaxCluster)
                TwoOptImprove(holes, tour);
            foreach (int idx in tour)
                ordered.Add(holes[idx]);
        }

        return ordered;
    }

    /// <summary>簇内最近邻贪心排序，返回孔索引的访问顺序（开路径）。</summary>
    private static List<int> NearestNeighborInCluster(
        List<Geometry.Drilling.DrillingPattern.Hole> holes, List<int> members, double startX, double startY)
    {
        int m = members.Count;
        var tour = new List<int>(m);
        var used = new bool[m];
        double px = startX, py = startY;
        for (int step = 0; step < m; step++)
        {
            int bestJ = -1;
            double bestD2 = double.MaxValue;
            for (int j = 0; j < m; j++)
            {
                if (used[j]) continue;
                int idx = members[j];
                double dx = holes[idx].X - px, dy = holes[idx].Y - py;
                double d2 = dx * dx + dy * dy;
                if (d2 < bestD2) { bestD2 = d2; bestJ = j; }
            }
            used[bestJ] = true;
            tour.Add(members[bestJ]);
            px = holes[members[bestJ]].X;
            py = holes[members[bestJ]].Y;
        }
        return tour;
    }

    /// <summary>2-opt 局部搜索优化开路径 TSP（原地翻转 tour）。仅对小规模簇调用。</summary>
    private static void TwoOptImprove(List<Geometry.Drilling.DrillingPattern.Hole> holes, List<int> tour)
    {
        int m = tour.Count;
        if (m < 4) return;
        bool improved = true;
        int maxPasses = 20;
        while (improved && maxPasses-- > 0)
        {
            improved = false;
            for (int i = 0; i < m - 1; i++)
            {
                for (int k = i + 2; k < m; k++)
                {
                    int a = tour[i], b = tour[i + 1];
                    int c = tour[k];
                    // 开路径：(c,d) 边仅当 k+1 < m 时存在
                    double before = Dist(holes, a, b);
                    double after = Dist(holes, a, c);
                    if (k + 1 < m)
                    {
                        int d = tour[k + 1];
                        before += Dist(holes, c, d);
                        after += Dist(holes, b, d);
                    }
                    if (after + 1e-9 < before)
                    {
                        tour.Reverse(i + 1, k - i);
                        improved = true;
                    }
                }
            }
        }
    }

    private static double Dist(List<Geometry.Drilling.DrillingPattern.Hole> holes, int a, int b)
    {
        double dx = holes[a].X - holes[b].X, dy = holes[a].Y - holes[b].Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>簇内 2-opt 优化的孔数上限（超过此数量仅用最近邻，避免 O(m²) 性能问题）。</summary>
    private const int TwoOptMaxCluster = 300;

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
    
    /// <summary>将钻孔移动序列转换为采样轨迹（确保孔间移动激光关闭）</summary>
    private static Core.PathPlanning.SampledTrajectory SampleDrillingTrajectory(
        List<HoleMove> moves, double jumpSpeedPlatform, double jumpSpeedGalvo, 
        double sampleRate, double dwellTimeMs)
    {
        if (moves.Count == 0) return new Core.PathPlanning.SampledTrajectory();
        
        var xs = new List<double>(1 << 16);
        var ys = new List<double>(1 << 16);
        var laser = new List<bool>(1 << 16);
        
        double dt = 1.0 / sampleRate;
        double rapidSpeed = Math.Min(
            Math.Sqrt(jumpSpeedPlatform * jumpSpeedPlatform + jumpSpeedGalvo * jumpSpeedGalvo),
            1000.0);  // 限制最大 1000 mm/s
        double step = rapidSpeed * dt;
        
        Vec2 cur = moves[0].Position;
        
        for (int i = 0; i < moves.Count; i++)
        {
            var move = moves[i];
            
            // 1. 空程移动到孔位（激光关闭）
            if (i > 0)
            {
                // 空程前：确保激光关闭
                if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
                {
                    xs.Add(cur.X);
                    ys.Add(cur.Y);
                    laser.Add(false);
                }
                
                // 空程移动到目标孔位
                double len = cur.DistanceTo(move.Position);
                if (len > 1e-12)
                {
                    double s = step;
                    while (s < len)
                    {
                        double t = s / len;
                        xs.Add(cur.X + (move.Position.X - cur.X) * t);
                        ys.Add(cur.Y + (move.Position.Y - cur.Y) * t);
                        laser.Add(false);  // 空程激光关闭
                        s += step;
                    }
                }
                
                // 空程结束：确保激光关闭
                if (xs.Count == 0 || xs[^1] != move.Position.X || ys[^1] != move.Position.Y)
                {
                    xs.Add(move.Position.X);
                    ys.Add(move.Position.Y);
                    laser.Add(false);
                }
            }
            
            // 2. 钻孔位置（激光开启，停留）
            int dwellSamples = (int)(dwellTimeMs / 1000.0 / dt);
            dwellSamples = Math.Max(dwellSamples, 1);  // 至少 1 个采样点
            
            for (int j = 0; j < dwellSamples; j++)
            {
                xs.Add(move.Position.X);
                ys.Add(move.Position.Y);
                laser.Add(true);  // 钻孔激光开启
            }
            
            cur = move.Position;
        }
        
        return new Core.PathPlanning.SampledTrajectory
        {
            SampleRate = sampleRate,
            X = xs.ToArray(),
            Y = ys.ToArray(),
            LaserOn = laser.ToArray()
        };
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
