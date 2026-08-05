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
    /// <summary>采样/仿真的轮廓数上限；超过时调用方应先 Decimate 抽稀</summary>
    public const int MaxSampleContours = 20_000;

    /// <summary>轮廓数超过此值时最近邻排序改用空间网格加速</summary>
    private const int GridOrderThreshold = 5_000;

    /// <param name="feedSpeed">进给速度 (mm/s)，激光出光时的加工速度</param>
    /// <param name="jumpSpeedPlatform">平台空移速度 (mm/s)，激光关闭时平台的快速移动速度</param>
    /// <param name="jumpSpeedGalvo">振镜空移速度 (mm/s)，激光关闭时振镜的快速扫描速度</param>
    /// <param name="sampleRate">采样频率 (Hz)</param>
    /// <param name="cornerAngleDeg">尖角保真阈值：内角小于该值的折线顶点强制作为采样点（顶点吸附），
    /// 避免尖角被等时采样弦切。默认 150°；设为 ≥180 可关闭该特性。</param>
    /// <param name="accelPlatform">平台加速度 (mm/s²)，用于计算平台加速段长度。默认 1000 mm/s²。</param>
    /// <param name="accelGalvo">振镜加速度 (mm/s²)，用于计算振镜加速段长度。默认 5000 mm/s²。</param>
    /// <param name="cornerFactor">拐角系数 (0.0-1.0)，控制尖角处的速度衰减。0=不减速，1=完全停止。默认 0.5。</param>
    /// <param name="decelPlatform">平台减速度 (mm/s²)，用于计算减速段长度。默认等于 accelPlatform。</param>
    public static SampledTrajectory Sample(IReadOnlyList<PathPolyline> polylines,
        double feedSpeed, double jumpSpeedPlatform, double jumpSpeedGalvo, double sampleRate, 
        double cornerAngleDeg = 150, double accelPlatform = 1000.0, double accelGalvo = 5000.0, 
        double cornerFactor = 0.5, double decelPlatform = 0)
    {
        // 计算合成空移速度（平台和振镜的向量和）
        // 物理模型：V_rapid = √(V_platform² + V_galvo²)
        double rapidSpeed = Math.Sqrt(jumpSpeedPlatform * jumpSpeedPlatform + jumpSpeedGalvo * jumpSpeedGalvo);
        
        // 限制最大速度，防止过快导致采样点过疏
        rapidSpeed = Math.Min(rapidSpeed, 1000.0);  // 最大 1000 mm/s
        
        var ordered = OrderByNearest(polylines);
        var xs = new List<double>(1 << 16);
        var ys = new List<double>(1 << 16);
        var laser = new List<bool>(1 << 16);
        double dt = 1.0 / sampleRate;
        bool snapCorners = cornerAngleDeg < 180;
        double cornerCos = Math.Cos(cornerAngleDeg * Math.PI / 180.0);

        Vec2 cur = ordered.Count > 0 ? ordered[0].Points[0] : Vec2.Zero;
        double residual = 0;   // 上一段剩余的采样相位距离

        foreach (var pl in ordered)
        {
            var pts = new List<Vec2>(pl.Points);
            if (pl.Closed && pts.Count > 1 && !pts[0].Equals(pts[^1])) pts.Add(pts[0]);

            // 空程前：确保当前位置的激光状态为关闭（防止轮廓段结束时激光误开启）
            if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
            {
                xs.Add(cur.X);
                ys.Add(cur.Y);
                laser.Add(false);  // 强制关闭激光
            }

            // 空程：当前位置 -> 轮廓起点（使用 rapidSpeed，恒定速度）
            residual = InterpolateSegment(cur, pts[0], rapidSpeed * dt, residual, xs, ys, laser, false);
            cur = pts[0];
            
            // 空程结束：确保到达轮廓起点前激光已关闭
            if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
            {
                xs.Add(cur.X);
                ys.Add(cur.Y);
                laser.Add(false);  // 强制关闭激光
            }

            // 轮廓段
            for (int i = 1; i < pts.Count; i++)
            {
                // 计算当前段的速度（考虑拐角系数）
                double segmentSpeed = feedSpeed;
                
                // 如果是尖角，根据 cornerFactor 衰减速度
                if (i > 1 && snapCorners && IsSharpCorner(pts[i - 2], pts[i - 1], pts[i], cornerCos))
                {
                    // 尖角处的速度 = feedSpeed * (1 - cornerFactor)
                    segmentSpeed = feedSpeed * (1.0 - cornerFactor);
                    
                    // 如果速度过低，至少保持 feedSpeed 的 10%
                    segmentSpeed = Math.Max(segmentSpeed, feedSpeed * 0.1);
                }
                
                residual = InterpolateSegment(cur, pts[i], segmentSpeed * dt, residual, xs, ys, laser, true);
                cur = pts[i];

                // 尖角保真：顶点非采样点时，若转角够尖则强制吸附该顶点并重置采样相位
                if (snapCorners && TryGetCornerNeighbor(pts, i, pl.Closed, out Vec2 nextPt)
                    && IsSharpCorner(pts[i - 1], cur, nextPt, cornerCos))
                {
                    if (xs.Count == 0 || xs[^1] != cur.X || ys[^1] != cur.Y)
                    { xs.Add(cur.X); ys.Add(cur.Y); laser.Add(true); }
                    residual = 0;   // 从尖角顶点重新计相位（局部牺牲等时性换取保真）
                }
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
    
    /// <summary>计算加速段和减速段的距离</summary>
    private static (double accelDist, double fullSpeedDist, double decelDist) 
        CalculateAccelDecelDistances(double currentSpeed, double targetSpeed, double accel, double decel)
    {
        // 加速距离：s = (v_final² - v_initial²) / (2 * a)
        double accelDist = (targetSpeed * targetSpeed - currentSpeed * currentSpeed) / (2.0 * accel);
        accelDist = Math.Max(0, accelDist);  // 防止负值
        
        // 减速距离：s = (v_final² - v_initial²) / (2 * a)
        double decelDist = (targetSpeed * targetSpeed - targetSpeed * targetSpeed) / (2.0 * decel);
        decelDist = Math.Max(0, decelDist);
        
        return (accelDist, 0, decelDist);  // 简化版本，fullSpeedDist 需要在调用处计算
    }
    
    /// <summary>沿线段以变步距采样（考虑加减速），返回跨段剩余相位</summary>
    private static double InterpolateSegmentWithAccel(
        Vec2 a, Vec2 b, double targetSpeed, double accel, double decel, double dt,
        double residual, List<double> xs, List<double> ys, List<bool> laser, bool laserOn,
        ref double currentSpeed)
    {
        double len = a.DistanceTo(b);
        if (len < 1e-12) return residual;
        
        // 计算加速段和减速段长度
        double accelDist = Math.Max(0, (targetSpeed * targetSpeed - currentSpeed * currentSpeed) / (2.0 * accel));
        double decelDist = Math.Max(0, (targetSpeed * targetSpeed) / (2.0 * decel));  // 减速到 0
        
        // 如果线段太短，无法完成加速和减速
        if (len < accelDist + decelDist)
        {
            // 按比例分配加速和减速
            double ratio = accel / (accel + decel);
            accelDist = len * ratio;
            decelDist = len * (1 - ratio);
        }
        
        double fullSpeedDist = len - accelDist - decelDist;
        fullSpeedDist = Math.Max(0, fullSpeedDist);
        
        // 分三段采样：加速段、匀速段、减速段
        double s = -residual;  // 当前弧长位置
        
        // 最小步长：防止速度过低时死循环
        double minStep = len / 10000.0;  // 至少走 1/10000 的线段长度
        
        // 1. 加速段
        while (s < accelDist && s < len)
        {
            // 计算当前位置的速度（线性加速）
            double localSpeed = currentSpeed + (accel * s / Math.Max(accelDist, 1e-9));
            localSpeed = Math.Min(localSpeed, targetSpeed);
            
            double step = Math.Max(localSpeed * dt, minStep);  // 保证最小步长
            s += step;
            
            if (s <= len)
            {
                double t = s / len;
                xs.Add(a.X + (b.X - a.X) * t);
                ys.Add(a.Y + (b.Y - a.Y) * t);
                laser.Add(laserOn);
            }
        }
        
        // 2. 匀速段
        while (s < accelDist + fullSpeedDist && s < len)
        {
            double step = Math.Max(targetSpeed * dt, minStep);
            s += step;
            
            if (s <= len)
            {
                double t = s / len;
                xs.Add(a.X + (b.X - a.X) * t);
                ys.Add(a.Y + (b.Y - a.Y) * t);
                laser.Add(laserOn);
            }
        }
        
        // 3. 减速段
        while (s < len)
        {
            double distInDecel = s - accelDist - fullSpeedDist;
            double localSpeed = targetSpeed - (decel * distInDecel / Math.Max(decelDist, 1e-9));
            localSpeed = Math.Max(localSpeed, 0);
            
            double step = Math.Max(localSpeed * dt, minStep);  // 保证最小步长
            s += step;
            
            if (s <= len)
            {
                double t = s / len;
                xs.Add(a.X + (b.X - a.X) * t);
                ys.Add(a.Y + (b.Y - a.Y) * t);
                laser.Add(laserOn);
            }
        }
        
        // 更新当前速度
        currentSpeed = targetSpeed;
        
        // 返回剩余相位
        return Math.Max(0, len - s);
    }

    /// <summary>取顶点 pts[i] 的「下一个」邻居（闭合轮廓在末点回绕到 pts[1]）；开折线末端无邻居返回 false。</summary>
    private static bool TryGetCornerNeighbor(List<Vec2> pts, int i, bool closed, out Vec2 next)
    {
        if (i + 1 <= pts.Count - 1) { next = pts[i + 1]; return true; }
        if (closed && pts.Count > 2) { next = pts[1]; return true; }   // pts[^1]==pts[0]，回绕取 pts[1]
        next = default; return false;
    }

    /// <summary>判断 prev→vertex→next 在 vertex 处的内角是否小于阈值（cos 大于阈值即为尖角）。</summary>
    private static bool IsSharpCorner(Vec2 prev, Vec2 vertex, Vec2 next, double cosThreshold)
    {
        double v1x = prev.X - vertex.X, v1y = prev.Y - vertex.Y;
        double v2x = next.X - vertex.X, v2y = next.Y - vertex.Y;
        double l1 = Math.Sqrt(v1x * v1x + v1y * v1y);
        double l2 = Math.Sqrt(v2x * v2x + v2y * v2y);
        if (l1 < 1e-12 || l2 < 1e-12) return false;
        double cos = (v1x * v2x + v1y * v2y) / (l1 * l2);
        return cos > cosThreshold;   // 内角 < 阈值 ⟹ 尖角
    }

    /// <summary>
    /// 空间均匀抽稀：按 √maxCount×√maxCount 网格分桶后逐桶轮流取样，
    /// 得到覆盖整个版图的代表性子集（轮廓数 ≤ maxCount 时原样返回）。
    /// </summary>
    public static List<PathPolyline> Decimate(IReadOnlyList<PathPolyline> polylines, int maxCount)
    {
        int n = polylines.Count;
        if (n <= maxCount)
            return polylines as List<PathPolyline> ?? new List<PathPolyline>(polylines);

        // 以首点为代表求全局包围盒
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        for (int i = 0; i < n; i++)
        {
            var pts = polylines[i].Points;
            if (pts.Count == 0) continue;
            Vec2 p = pts[0];
            if (p.X < minX) minX = p.X;
            if (p.X > maxX) maxX = p.X;
            if (p.Y < minY) minY = p.Y;
            if (p.Y > maxY) maxY = p.Y;
        }
        if (minX > maxX) return new List<PathPolyline>();

        int dim = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(maxCount)));
        double cw = Math.Max((maxX - minX) / dim, 1e-9);
        double ch = Math.Max((maxY - minY) / dim, 1e-9);
        int bucketCount = dim * dim;

        // 计数排序分桶（避免 List<List<>> 大量小分配）
        var cellOf = new int[n];
        var counts = new int[bucketCount + 1];
        for (int i = 0; i < n; i++)
        {
            var pts = polylines[i].Points;
            if (pts.Count == 0) { cellOf[i] = -1; continue; }
            int cx = Math.Clamp((int)((pts[0].X - minX) / cw), 0, dim - 1);
            int cy = Math.Clamp((int)((pts[0].Y - minY) / ch), 0, dim - 1);
            int cell = cy * dim + cx;
            cellOf[i] = cell;
            counts[cell + 1]++;
        }
        for (int b = 0; b < bucketCount; b++) counts[b + 1] += counts[b];
        var grouped = new int[n];
        var cursor = (int[])counts.Clone();
        for (int i = 0; i < n; i++)
        {
            if (cellOf[i] < 0) continue;
            grouped[cursor[cellOf[i]]++] = i;
        }

        // 逐桶轮流取样直至满额，保证空间均匀覆盖
        var result = new List<PathPolyline>(maxCount);
        for (int pass = 0; result.Count < maxCount; pass++)
        {
            bool any = false;
            for (int b = 0; b < bucketCount && result.Count < maxCount; b++)
            {
                int idx = counts[b] + pass;
                if (idx >= cursor[b]) continue;
                result.Add(polylines[grouped[idx]]);
                any = true;
            }
            if (!any) break;
        }
        return result;
    }

    /// <summary>贪心最近邻排序（可整体反向折线）；大数据自动切换网格加速版</summary>
    private static List<PathPolyline> OrderByNearest(IReadOnlyList<PathPolyline> input)
    {
        var remaining = input.Where(p => p.Points.Count > 1).ToList();
        if (remaining.Count > GridOrderThreshold) return OrderByGrid(remaining);
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

    /// <summary>
    /// 网格加速的贪心最近邻排序：把各轮廓端点挂入均匀空间网格，
    /// 每步从当前所在网格单元向外环形扩展查找最近端点，复杂度近似 O(K)。
    /// 排序质量与 O(K²) 贪心接近，适用于上万级轮廓。
    /// </summary>
    private static List<PathPolyline> OrderByGrid(List<PathPolyline> polys)
    {
        int n = polys.Count;
        // 端点表：闭合轮廓只登记首点；开折线登记首点与尾点（尾点表示反向进入）
        var ex = new List<double>(n * 2);
        var ey = new List<double>(n * 2);
        var epoly = new List<int>(n * 2);
        var etail = new List<bool>(n * 2);
        for (int i = 0; i < n; i++)
        {
            var p = polys[i];
            ex.Add(p.Points[0].X); ey.Add(p.Points[0].Y); epoly.Add(i); etail.Add(false);
            if (!p.Closed)
            { ex.Add(p.Points[^1].X); ey.Add(p.Points[^1].Y); epoly.Add(i); etail.Add(true); }
        }
        int m = ex.Count;

        // 端点全局包围盒 → 均匀网格
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

        // 计数排序建桶：starts[cell]..starts[cell+1] 区间是该单元的端点索引
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

        var usedPoly = new bool[n];      // 轮廓级懒删除（同一折线两个端点共用）
        var ordered = new List<PathPolyline>(n);
        double px = 0, py = 0;           // 当前位置（从原点出发，与原实现一致）
        double minCell = Math.Min(cw, ch);

        for (int step = 0; step < n; step++)
        {
            int ccx = Math.Clamp((int)((px - minX) / cw), 0, dim - 1);
            int ccy = Math.Clamp((int)((py - minY) / ch), 0, dim - 1);
            int best = -1;
            double bestD2 = double.MaxValue;

            for (int r = 0; r <= 2 * dim; r++)
            {
                // 已找到候选且更外环不可能更近时提前终止
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
                        if (!edgeRow && cx != xlo && cx != xhi) continue;   // 只扫环边
                        int cell = cy * dim + cx;
                        for (int t = starts[cell]; t < starts[cell + 1]; t++)
                        {
                            int k = items[t];
                            if (usedPoly[epoly[k]]) continue;
                            double dx = ex[k] - px, dy = ey[k] - py;
                            double d2 = dx * dx + dy * dy;
                            if (d2 < bestD2) { bestD2 = d2; best = k; }
                        }
                    }
                }
                if (r >= dim && best >= 0) break;
            }
            if (best < 0) break;   // 理论不可达（除非全部已用）

            int pi = epoly[best];
            usedPoly[pi] = true;
            var pick = polys[pi];
            if (etail[best])
            {
                // 从尾端进入：整体反向（与原实现一致）
                var rp = new PathPolyline { Closed = pick.Closed, Layer = pick.Layer };
                for (int i = pick.Points.Count - 1; i >= 0; i--) rp.Points.Add(pick.Points[i]);
                pick = rp;
            }
            ordered.Add(pick);
            px = pick.Closed ? pick.Points[0].X : pick.Points[^1].X;
            py = pick.Closed ? pick.Points[0].Y : pick.Points[^1].Y;
        }
        return ordered;
    }
}
