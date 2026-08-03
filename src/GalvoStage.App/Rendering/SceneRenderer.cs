using System;
using System.Collections.Generic;
using GalvoStage.App.ViewModels;
using GalvoStage.Core.Geometry;
using SkiaSharp;

namespace GalvoStage.App.Rendering;

/// <summary>世界坐标(mm, Y向上) ↔ 屏幕坐标 变换</summary>
public sealed class ViewTransform
{
    public double Scale { get; private set; } = 5;      // px per mm
    public double OffsetX { get; private set; }
    public double OffsetY { get; private set; }

    public SKPoint ToScreen(double wx, double wy)
        => new((float)(OffsetX + wx * Scale), (float)(OffsetY - wy * Scale));

    public (double x, double y) ToWorld(float sx, float sy)
        => ((sx - OffsetX) / Scale, (OffsetY - sy) / Scale);

    public void Pan(double dxPx, double dyPx) { OffsetX += dxPx; OffsetY += dyPx; }

    public void ZoomAt(float sx, float sy, double factor)
    {
        var (wx, wy) = ToWorld(sx, sy);
        Scale = Math.Clamp(Scale * factor, 0.05, 5000);
        OffsetX = sx - wx * Scale;
        OffsetY = sy + wy * Scale;
    }

    public void Fit(double minX, double minY, double maxX, double maxY, float width, float height)
    {
        double w = Math.Max(maxX - minX, 1e-6), h = Math.Max(maxY - minY, 1e-6);
        Scale = Math.Min((width - 80) / w, (height - 80) / h);
        Scale = Math.Clamp(Scale, 0.05, 5000);
        double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
        OffsetX = width / 2 - cx * Scale;
        OffsetY = height / 2 + cy * Scale;
    }

    /// <summary>将世界原点 (0,0) 置于画布中心（保持当前缩放）</summary>
    public void CenterOrigin(float width, float height)
    {
        OffsetX = width / 2;
        OffsetY = height / 2;
    }
}

/// <summary>SkiaSharp 场景绘制：图形/分解路径/实时加工/FOV/误差曲线</summary>
public static class SceneRenderer
{
    private static readonly SKColor BgColor = new(0x17, 0x17, 0x1C);
    private static readonly SKColor GridColor = new(0x2A, 0x2A, 0x33);
    private static readonly SKColor GeomColor = new(0x6E, 0x6E, 0x7A);
    private static readonly SKColor StageColor = new(0x3A, 0x86, 0xFF);
    private static readonly SKColor StageActColor = new(0x2E, 0xC4, 0xE6);
    private static readonly SKColor SpotColor = new(0xFF, 0x3B, 0x30);
    private static readonly SKColor FovColor = new(0x2E, 0xCC, 0x71);
    private static readonly SKColor GalvoLineColor = new(0xFF, 0xD6, 0x0A);
    private static readonly SKColor DrillPointColor = new(0xFF, 0x5C, 0xB8);   // 紫红点 - 钻孔位置
    private static readonly SKColor DrillTrajColor = new(0xF9, 0xBE, 0x5F);     // 橙色线 - 钻孔顺序
    private static readonly SKColor RulerBg = new(0x1A, 0x1A, 0x24);
    private static readonly SKColor RulerLine = new(0x4A, 0x4E, 0x68);
    private static readonly SKColor AxisColor = new(0x3F, 0x7A, 0x5E);   // XY 坐标轴（绿色）
    private static readonly SKColor RulerText = new SKColor(0x7D, 0x81, 0x95);
    private static readonly SKColor MouseHlColor = new(0x4C, 0x8D, 0xFF);
    private static readonly SKColor MouseBgColor = new SKColor(0x1F, 0x26, 0x3E).WithAlpha(240);
    private static readonly SKColor MouseTextColor = new(0xEF, 0xF2, 0xFA);

    // ---------------- LOD 参数（超大数据时启用，小数据零回归） ----------------
    private const int LodContourThreshold = 50_000;   // 轮廓数超过此值才启用 LOD 绘制
    private const int FullPathBudget = 20_000;        // 单帧完整绘制的轮廓上限，超出退化为点
    private const float SubPixelSizePx = 2f;          // 屏幕尺寸小于该像素的轮廓只画点
    private const int DedupCellPx = 2;                // 点云屏幕去重网格粒度(px)
    private const int PathFlushBatch = 4_096;         // 共享 SKPath 每积累多少条轮廓 flush 一次

    // 复用缓冲（仅 UI 线程渲染时使用，避免每帧大分配）
    private static byte[] _dedupCells = Array.Empty<byte>();

    public static void DrawScene(SKCanvas canvas, MainViewModel vm, ViewTransform vt, float width, float height, (double X, double Y)? mouseWorld = null)
    {
        canvas.Clear(BgColor);
        DrawGrid(canvas, vt, width, height);
        DrawRulers(canvas, vt, width, height);

        using var paint = new SKPaint { IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = 1f };

        // 1) 原始 DXF 图形（超大数据走视口裁剪 + 点云 LOD）
        paint.Color = GeomColor;
        paint.StrokeWidth = 1f;
        var cache = vm.GeometryCache;
        if (cache != null && vm.Polylines.Count > LodContourThreshold && cache.Count == vm.Polylines.Count)
            DrawPolylinesLod(canvas, paint, vt, vm.Polylines, cache, width, height);
        else
            foreach (var pl in vm.Polylines)
                DrawPolyline(canvas, paint, vt, pl);

        var plan = vm.Plan;
        var sim = vm.Sim;
        var drillPattern = vm.DrillingPattern;
        var drillTraj = vm.DrillingTrajectory;

        // 1.5) PCB 钻孔点（紫红点云，带视口裁剪 + 屏幕网格去重 LOD）
        if (drillPattern != null && drillPattern.Holes.Count > 0)
            DrawDrillingPattern(canvas, paint, vt, drillPattern, width, height);

        // 2) 平台低频路径（分解结果）
        if (plan != null)
        {
            paint.Color = StageColor.WithAlpha(160);
            paint.StrokeWidth = 1.6f;
            DrawSampled(canvas, paint, vt, plan.StageX, plan.StageY, 0, plan.Count);
        }

        // 3) PCB 钻孔顺序轨迹（橙色连线）
        if (drillTraj != null && drillTraj.Moves.Count > 1)
            DrawDrillingTrajectory(canvas, paint, vt, drillTraj);

        // 4) 实时加工轨迹
        if (sim != null && sim.Index > 1)
        {
            int n = sim.Index;
            // 平台实际轨迹（淡青色）
            paint.Color = StageActColor.WithAlpha(90);
            paint.StrokeWidth = 1f;
            DrawSampled(canvas, paint, vt, sim.StageActX, sim.StageActY, 0, n);

            // 激光落点轨迹（红=出光）
            DrawSpotTrace(canvas, vt, plan!, sim, n);

            // 4) 当前状态标记
            DrawLiveMarkers(canvas, vt, vm, sim);
        }

        DrawMouseCursorCrosshair(canvas, vt, width, height, mouseWorld);
        DrawLegend(canvas, width);
    }

    // ------------------ XY 标尺与鼠标位置高亮 --------------------

    /// <summary>绘制 XY 轴标尺 + 网格刻度线（X 轴在顶部，Y 轴在左侧）</summary>
    private static void DrawRulers(SKCanvas canvas, ViewTransform vt, float width, float height)
    {
        var (wx0, wy1) = vt.ToWorld(0, 0);
        var (wx1, wy0) = vt.ToWorld(width, height);

        using var paint = new SKPaint { IsAntialias = true };
        using var textPaint = new SKPaint
        {
            Color = RulerText,
            TextSize = 11f,
            IsAntialias = true,
            Typeface = SKTypeface.FromFamilyName("Microsoft YaHei UI", SKFontStyle.Normal)
        };

        // X 轴标尺 (顶部 40px) - 深色背景
        canvas.DrawRect(0, 0, width, 40, new SKPaint { Color = RulerBg });
        // X 轴边框线
        using var border = new SKPaint { Color = RulerLine.WithAlpha(120), StrokeWidth = 1.5f };
        canvas.DrawLine(0, 40, width, 40, border);

        int maxMajorTicks = Math.Max(3, (int)(width / 40));
        double xSpan = wx1 - wx0;
        if (xSpan <= 1e-9) return;
        double xStep = xSpan / maxMajorTicks;
        double xNice = NiceStep(xStep);

        for (double x = Math.Floor(wx0 / xNice) * xNice; x <= wx1 + xNice/2; x += xNice)
        {
            var pos = vt.ToScreen(x, 0);
            // 主刻度（长）+ 数字标签 - 紧贴标尺下沿向上
            paint.Color = RulerLine;
            paint.StrokeWidth = 2;
            canvas.DrawLine(pos.X, 40, pos.X, 28, paint);

            // 次刻度（短）
            double subX = x + xNice * 0.2;
            if (subX <= wx1)
            {
                var subPos = vt.ToScreen(subX, 0);
                canvas.DrawLine(subPos.X, 40, subPos.X, 34, 
                    new SKPaint { Color = RulerLine, StrokeWidth = 1 });
            }

            // 坐标文字：居中对齐并带深色底
            string txt = $"{x:+0.#;-0.#}";
            float tw = textPaint.MeasureText(txt);
            canvas.DrawText(txt, pos.X - tw / 2, 22, textPaint);
        }

        // Y 轴标尺 (左侧 40px) - 深色背景
        canvas.DrawRect(0, 0, 40, height, new SKPaint { Color = RulerBg });
        // Y 轴边框线
        using var borderY = new SKPaint { Color = RulerLine.WithAlpha(120), StrokeWidth = 1.5f };
        canvas.DrawLine(40, 0, 40, height, borderY);

        int yCount = Math.Min(100, (int)(height / 28));
        double ySpan = wy1 - wy0;
        if (ySpan <= 1e-9) return;
        double yStep = ySpan / yCount;
        double yNice = NiceStep(yStep);

        for (double y = Math.Floor(wy0 / yNice) * yNice; y <= wy1 + yNice/2; y += yNice)
        {
            var pos = vt.ToScreen(0, y);
            paint.Color = RulerLine;
            paint.StrokeWidth = 2;
            canvas.DrawLine(40, pos.Y, 28, pos.Y, paint); // 紧贴标尺右沿向左

            // 次刻度（短）
            double subY = y + yNice * 0.2;
            if (subY <= wy1)
            {
                var subPos = vt.ToScreen(0, subY);
                canvas.DrawLine(40, subPos.Y, 34, subPos.Y, 
                    new SKPaint { Color = RulerLine, StrokeWidth = 1 });
            }

            // 坐标文字：居中对齐
            string txt = $"{y:+0.#;-0.#}";
            canvas.DrawText(txt, 2, pos.Y + 4, textPaint);
        }

        // 坐标轴标签
        using var axisLabel = new SKPaint 
        { 
            Color = RulerLine.WithAlpha(200), 
            TextSize = 12f, 
            IsAntialias = true 
        };
        canvas.DrawText("X 轴 (mm)", width / 2 - textPaint.MeasureText("X 轴 (mm)") / 2, 12, axisLabel);       // 顶部居中
        canvas.DrawText("Y 轴 (mm)", 2, 54, axisLabel);     // 左侧标尺顶端下方
    }

    /// <summary>鼠标位置十字线与坐标数值（带 Alpha 通道支持）</summary>
    public static void DrawMouseCursorCrosshair(
        SKCanvas canvas, ViewTransform vt, float width, float height, (double X, double Y)? mouseWorld)
    {
        if (mouseWorld == null || double.IsNaN(mouseWorld.Value.X) || double.IsNaN(mouseWorld.Value.Y))
            return;

        var p = vt.ToScreen(mouseWorld.Value.X, mouseWorld.Value.Y);
        if (p.X < -100 || p.X > width + 100 || p.Y < -100 || p.Y > height + 100)
            return; // 超出视口太远则不绘

        // 仅在十字中心点显示
        using var centerDot = new SKPaint
        {
            Color = MouseHlColor.WithAlpha(255),
            Style = SKPaintStyle.Fill
        };
        canvas.DrawCircle(p.X, p.Y, 3.5f, centerDot);

        // X 轴方向：鼠标在画布 X 范围内时，在顶部 X 标尺上显示实时垂直刻度
        if (p.X >= 40 && p.X <= width)
        {
            using var vertLine = new SKPaint
            {
                Color = MouseHlColor.WithAlpha(220),
                StrokeWidth = 2f,
                IsAntialias = true
            };
            // 在顶部标尺区域内画一小段垂直刻度线（对齐鼠标 X 位置）
            canvas.DrawLine(p.X, 40, p.X, 20, vertLine);
        }

        // Y 轴方向：鼠标在画布 Y 范围内时，在左侧 Y 标尺上显示实时水平刻度
        if (p.Y >= 40 && p.Y <= height)
        {
            using var horizLine = new SKPaint
            {
                Color = MouseHlColor.WithAlpha(220),
                StrokeWidth = 2f,
                IsAntialias = true
            };
            // 在左侧标尺区域内画一小段水平刻度线（对齐鼠标 Y 位置）
            canvas.DrawLine(40, p.Y, 20, p.Y, horizLine);
        }

        // 坐标气泡框优化：深色渐变背景 + 圆角矩形
        string label = $"X={mouseWorld.Value.X:F3}mm  Y={mouseWorld.Value.Y:F3}mm";
        using var textPaint = new SKPaint
        {
            Color = MouseTextColor,
            TextSize = 12f,
            IsAntialias = true,
            Typeface = SKTypeface.FromFamilyName("Microsoft YaHei UI", SKFontStyle.Normal)
        };

        // 测量文字宽度
        float textWidth = textPaint.MeasureText(label);
        float textHeight = textPaint.TextSize;

        // 气泡位置和尺寸（十字线左侧上方）
        float w = textWidth + 14f;
        float h = textHeight + 12f;
        //float bx = Math.Max(48f, Math.Min(p.X - w - 4, width - w - 48f));
        //float by = Math.Max(48f, Math.Min(p.Y - h - 6, height - h - 6f));
        float bx = width-190;
        float by =  height-35;

        // 填充背景（深色圆角背景）
        using var bgPaint = new SKPaint { Color = MouseBgColor, Style = SKPaintStyle.Fill };
        canvas.DrawRoundRect(bx, by, w, h, 6, 6, bgPaint);

        // 气泡边框（亮色高光）
        using var strokePaint = new SKPaint 
        { 
            Color = MouseHlColor.WithAlpha(255), 
            StrokeWidth = 1.5f, 
            IsAntialias = true 
        };
        canvas.DrawRoundRect(bx, by, w, h, 6, 6, strokePaint);

        // 添加阴影效果（可选增强版）
        using var shadowPaint = new SKPaint 
        { 
            Color = MouseBgColor.WithAlpha(180),
            Style = SKPaintStyle.Fill
        };
        canvas.DrawRoundRect(bx + 2f, by + 2f, w, h, 6, 6, shadowPaint);

        // 文字居中显示
        textPaint.Color = MouseTextColor;
        canvas.DrawText(label, bx + 4, by + h - 6, textPaint);
    }

    /// <summary>背景网格 + XY 坐标轴（世界原点十字轴线，绿色高亮）</summary>
    private static void DrawGrid(SKCanvas canvas, ViewTransform vt, float width, float height)
    {
        using var paint = new SKPaint { Color = GridColor, StrokeWidth = 1 };
        double stepMm = NiceStep(50.0 / vt.Scale);
        var (wx0, wy1) = vt.ToWorld(0, 0);
        var (wx1, wy0) = vt.ToWorld(width, height);
        for (double x = Math.Floor(wx0 / stepMm) * stepMm; x <= wx1; x += stepMm)
        {
            var p = vt.ToScreen(x, 0);
            canvas.DrawLine(p.X, 0, p.X, height, paint);
        }
        for (double y = Math.Floor(wy0 / stepMm) * stepMm; y <= wy1; y += stepMm)
        {
            var p = vt.ToScreen(0, y);
            canvas.DrawLine(0, p.Y, width, p.Y, paint);
        }

        // XY 坐标轴：世界 X=0 / Y=0 贯穿画布的绿色轴线
        using var axis = new SKPaint { Color = AxisColor.WithAlpha(180), StrokeWidth = 1.6f, IsAntialias = true };
        var o = vt.ToScreen(0, 0);
        if (o.X >= 0 && o.X <= width)
            canvas.DrawLine(o.X, 0, o.X, height, axis);   // Y 轴（垂直）
        if (o.Y >= 0 && o.Y <= height)
            canvas.DrawLine(0, o.Y, width, o.Y, axis);    // X 轴（水平）

        // 原点标记：小圆圈 + 十字
        if (o.X >= -20 && o.X <= width + 20 && o.Y >= -20 && o.Y <= height + 20)
        {
            using var originPaint = new SKPaint
            {
                Color = AxisColor,
                StrokeWidth = 1.6f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            };
            canvas.DrawCircle(o.X, o.Y, 5f, originPaint);
            canvas.DrawLine(o.X - 10, o.Y, o.X + 10, o.Y, originPaint);
            canvas.DrawLine(o.X, o.Y - 10, o.X, o.Y + 10, originPaint);
        }
    }

    private static double NiceStep(double raw)
    {
        double mag = Math.Pow(10, Math.Floor(Math.Log10(Math.Max(raw, 1e-9))));
        double r = raw / mag;
        return (r < 1.5 ? 1 : r < 3.5 ? 2 : r < 7.5 ? 5 : 10) * mag;
    }

    private static void DrawPolyline(SKCanvas canvas, SKPaint paint, ViewTransform vt, PathPolyline pl)
    {
        if (pl.Points.Count < 2) return;
        using var path = new SKPath();
        var p0 = vt.ToScreen(pl.Points[0].X, pl.Points[0].Y);
        path.MoveTo(p0);
        for (int i = 1; i < pl.Points.Count; i++)
            path.LineTo(vt.ToScreen(pl.Points[i].X, pl.Points[i].Y));
        if (pl.Closed) path.Close();
        canvas.DrawPath(path, paint);
    }

    /// <summary>
    /// 超大数据 LOD 绘制：视口外轮廓直接跳过；视口内按屏幕尺寸分级——
    /// 亚像素轮廓合并为点云（带屏幕网格去重），较大轮廓合并进共享 SKPath 分批绘制，
    /// 且受全帧路径预算限制，超预算部分退化为点。帧开销由视口内容而非总量决定。
    /// 收集阶段并行扫描包围盒数组（去重网格允许良性竞争，仅可能产生少量重复点），
    /// Skia 绘制仍在调用线程串行执行。
    /// </summary>
    private static void DrawPolylinesLod(SKCanvas canvas, SKPaint paint, ViewTransform vt,
        List<PathPolyline> polylines, SceneGeometryCache cache, float width, float height)
    {
        // 视口的世界坐标范围（世界 Y 向上：屏幕顶部是 maxY）
        var (wl, wt) = vt.ToWorld(0, 0);
        var (wr, wb) = vt.ToWorld(width, height);
        float vx0 = (float)wl, vx1 = (float)wr, vy0 = (float)wb, vy1 = (float)wt;
        double scale = vt.Scale;

        // 点云去重网格（覆盖整个视口，DedupCellPx 一格）
        int gw = (int)(width / DedupCellPx) + 2;
        int gh = (int)(height / DedupCellPx) + 2;
        int cells = gw * gh;
        if (_dedupCells.Length < cells) _dedupCells = new byte[cells];
        else Array.Clear(_dedupCells, 0, cells);
        var dedup = _dedupCells;

        float[] minX = cache.MinX, minY = cache.MinY, maxX = cache.MaxX, maxY = cache.MaxY;
        int n = cache.Count;
        var bigAll = new List<int>();          // 需完整绘制的轮廓索引
        var ptsAll = new List<SKPoint>();      // 点云
        object gate = new();

        // 按块分区并行，块内跑紧循环，避免逐元素委托开销
        const int chunk = 1 << 16;
        int chunkCount = (n + chunk - 1) / chunk;
        double offX = vt.OffsetX, offY = vt.OffsetY;

        System.Threading.Tasks.Parallel.For(0, chunkCount,
            () => (big: new List<int>(), pts: new List<SKPoint>()),
            (ci, _, acc) =>
            {
                int i0 = ci * chunk, i1 = Math.Min(i0 + chunk, n);
                for (int i = i0; i < i1; i++)
                {
                    // 视口裁剪
                    if (maxX[i] < vx0 || minX[i] > vx1 || maxY[i] < vy0 || minY[i] > vy1) continue;

                    float sizeWorld = Math.Max(maxX[i] - minX[i], maxY[i] - minY[i]);
                    if (sizeWorld * scale >= SubPixelSizePx)
                    {
                        acc.big.Add(i);
                        continue;
                    }
                    // 点云：包围盒中心 → 屏幕坐标，2px 网格去重（并行竞争无害）
                    float sx = (float)(offX + (minX[i] + maxX[i]) * 0.5 * scale);
                    float sy = (float)(offY - (minY[i] + maxY[i]) * 0.5 * scale);
                    if (sx < 0 || sx >= width || sy < 0 || sy >= height) continue;
                    int cell = (int)(sy / DedupCellPx) * gw + (int)(sx / DedupCellPx);
                    if (dedup[cell] != 0) continue;
                    dedup[cell] = 1;
                    acc.pts.Add(new SKPoint(sx, sy));
                }
                return acc;
            },
            acc =>
            {
                lock (gate)
                {
                    bigAll.AddRange(acc.big);
                    ptsAll.AddRange(acc.pts);
                }
            });

        // 完整绘制（预算内），超预算部分退化为包围盒中心点。
        // 按索引排序保证帧间确定性，避免超预算时闪烁。
        bigAll.Sort();
        using var batch = new SKPath();
        int batched = 0;
        for (int k = 0; k < bigAll.Count; k++)
        {
            int i = bigAll[k];
            if (k < FullPathBudget)
            {
                AppendPolyline(batch, vt, polylines[i]);
                if (++batched >= PathFlushBatch)
                {
                    canvas.DrawPath(batch, paint);
                    batch.Reset();
                    batched = 0;
                }
            }
            else
            {
                ptsAll.Add(vt.ToScreen((minX[i] + maxX[i]) * 0.5, (minY[i] + maxY[i]) * 0.5));
            }
        }
        if (batched > 0) canvas.DrawPath(batch, paint);
        if (ptsAll.Count > 0) FlushPoints(canvas, paint, ptsAll.ToArray(), ptsAll.Count);
    }

    /// <summary>
    /// PCB 钻孔渲染：按每个孔的真实孔径（Diameter）画圆轮廓，还原原始图形。
    /// 视口裁剪（含孔半径，大孔中心在视口外仍绘制）+ 屏幕尺寸分级 LOD：
    /// 屏幕半径 ≥ 1.5px 的孔画真实孔径圆；亚像素小孔退化为点云（屏幕网格去重），600 万孔也能流畅平移缩放。
    /// </summary>
    private static void DrawDrillingPattern(SKCanvas canvas, SKPaint paint, ViewTransform vt,
        Core.Geometry.Drilling.DrillingPattern pattern, float width, float height)
    {
        var holes = pattern.Holes;
        int n = holes.Count;

        // 视口世界范围
        var (wl, wt) = vt.ToWorld(0, 0);
        var (wr, wb) = vt.ToWorld(width, height);
        double vx0 = wl, vx1 = wr, vy0 = wb, vy1 = wt;
        double scale = vt.Scale, offX = vt.OffsetX, offY = vt.OffsetY;

        // 屏幕网格去重（复用 LOD 缓冲）——仅用于亚像素小孔的点云
        int gw = (int)(width / DedupCellPx) + 2;
        int gh = (int)(height / DedupCellPx) + 2;
        int cells = gw * gh;
        if (_dedupCells.Length < cells) _dedupCells = new byte[cells];
        else Array.Clear(_dedupCells, 0, cells);
        var dedup = _dedupCells;

        const float MinCircleRadiusPx = 1.5f;   // 小于此屏幕半径的孔退化为点
        var circles = new List<(SKPoint c, float r)>(1024);
        var pts = new List<SKPoint>(Math.Min(n, 200_000));
        bool circleCapped = false;
        for (int i = 0; i < n; i++)
        {
            var h = holes[i];
            double rW = h.Diameter * 0.5;                 // 世界半径 (mm)
            // 视口裁剪：计入孔半径，使中心在视口外但圆弧可见的大孔仍被绘制
            if (h.X + rW < vx0 || h.X - rW > vx1 || h.Y + rW < vy0 || h.Y - rW > vy1) continue;

            float sx = (float)(offX + h.X * scale);
            float sy = (float)(offY - h.Y * scale);
            float rPx = (float)(rW * scale);              // 屏幕半径 (px)
            if (!circleCapped && rPx >= MinCircleRadiusPx)
            {
                circles.Add((new SKPoint(sx, sy), rPx));
                if (circles.Count > 60_000) circleCapped = true;   // 极端情况下停止收集，防卡顿
            }
            else
            {
                if (sx < 0 || sx >= width || sy < 0 || sy >= height) continue;
                int cell = (int)(sy / DedupCellPx) * gw + (int)(sx / DedupCellPx);
                if (dedup[cell] != 0) continue;
                dedup[cell] = 1;
                pts.Add(new SKPoint(sx, sy));
            }
        }

        var saved = (paint.Color, paint.Style, paint.StrokeWidth, paint.IsAntialias);
        // 亚像素小孔：点云
        if (pts.Count > 0)
        {
            paint.Color = DrillPointColor;
            FlushPoints(canvas, paint, pts.ToArray(), pts.Count);
        }
        // 可见孔：按真实孔径画圆轮廓
        if (circles.Count > 0)
        {
            paint.Color = DrillPointColor;
            paint.Style = SKPaintStyle.Stroke;
            paint.StrokeWidth = 1.2f;
            paint.IsAntialias = true;
            foreach (var (c, r) in circles)
                canvas.DrawCircle(c, r, paint);
        }
        (paint.Color, paint.Style, paint.StrokeWidth, paint.IsAntialias) = saved;
    }

    /// <summary>钻孔顺序轨迹：按规划顺序连线（降采样防卡顿）</summary>
    private static void DrawDrillingTrajectory(SKCanvas canvas, SKPaint paint, ViewTransform vt,
        Core.Drilling.DrillPlanner.DrillingTrajectory traj)
    {
        var moves = traj.Moves;
        int n = moves.Count;
        if (n < 2) return;
        int stride = Math.Max(1, n / 20000);
        var saved = (paint.Color, paint.StrokeWidth);
        paint.Color = DrillTrajColor.WithAlpha(140);
        paint.StrokeWidth = 1f;
        using var path = new SKPath();
        path.MoveTo(vt.ToScreen(moves[0].Position.X, moves[0].Position.Y));
        for (int i = stride; i < n; i += stride)
            path.LineTo(vt.ToScreen(moves[i].Position.X, moves[i].Position.Y));
        canvas.DrawPath(path, paint);
        (paint.Color, paint.StrokeWidth) = saved;
    }

    private static void FlushPoints(SKCanvas canvas, SKPaint paint, SKPoint[] buf, int count)
    {
        var savedCap = paint.StrokeCap;
        float savedWidth = paint.StrokeWidth;
        bool savedAA = paint.IsAntialias;
        paint.StrokeCap = SKStrokeCap.Square;
        paint.StrokeWidth = 1.6f;
        paint.IsAntialias = false;   // 亚像素方点无需抗锯齿，可显著降低批量绘制开销
        if (count < buf.Length)
        {
            var slice = new SKPoint[count];
            Array.Copy(buf, slice, count);
            canvas.DrawPoints(SKPointMode.Points, slice, paint);
        }
        else
        {
            canvas.DrawPoints(SKPointMode.Points, buf, paint);
        }
        paint.StrokeCap = savedCap;
        paint.StrokeWidth = savedWidth;
        paint.IsAntialias = savedAA;
    }

    private static void AppendPolyline(SKPath path, ViewTransform vt, PathPolyline pl)
    {
        var pts = pl.Points;
        if (pts.Count < 2) return;
        path.MoveTo(vt.ToScreen(pts[0].X, pts[0].Y));
        for (int i = 1; i < pts.Count; i++)
            path.LineTo(vt.ToScreen(pts[i].X, pts[i].Y));
        if (pl.Closed) path.Close();
    }

    private static void DrawSampled(SKCanvas canvas, SKPaint paint, ViewTransform vt,
        double[] xs, double[] ys, int start, int end)
    {
        int n = end - start;
        if (n < 2) return;
        int stride = Math.Max(1, n / 20000);   // 降采样，避免超长路径卡顿
        using var path = new SKPath();
        path.MoveTo(vt.ToScreen(xs[start], ys[start]));
        for (int i = start + stride; i < end; i += stride)
            path.LineTo(vt.ToScreen(xs[i], ys[i]));
        canvas.DrawPath(path, paint);
    }

    private static void DrawSpotTrace(SKCanvas canvas, ViewTransform vt,
        Core.PathPlanning.DecomposeResult plan, Core.Simulation.LinkageSimulator sim, int n)
    {
        using var paint = new SKPaint
        { IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = 2f, Color = SpotColor };
        int stride = Math.Max(1, n / 30000);
        using var path = new SKPath();
        bool penDown = false;
        for (int i = 0; i < n; i += stride)
        {
            if (plan.Raw.LaserOn[i])
            {
                var p = vt.ToScreen(sim.SpotX[i], sim.SpotY[i]);
                if (!penDown) { path.MoveTo(p); penDown = true; }
                else path.LineTo(p);
            }
            else penDown = false;
        }
        canvas.DrawPath(path, paint);
    }

    private static void DrawLiveMarkers(SKCanvas canvas, ViewTransform vt, MainViewModel vm,
        Core.Simulation.LinkageSimulator sim)
    {
        // 振镜视场框（以平台实际位置为中心）
        float fovPx = (float)(vm.GalvoFov * vt.Scale);
        var stagePos = vt.ToScreen(sim.CurStageActX, sim.CurStageActY);
        using var fovPaint = new SKPaint
        {
            IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = 1.4f,
            Color = FovColor.WithAlpha(200), PathEffect = SKPathEffect.CreateDash(new float[] { 6, 4 }, 0)
        };
        canvas.DrawRect(stagePos.X - fovPx, stagePos.Y - fovPx, fovPx * 2, fovPx * 2, fovPaint);

        // 平台位置十字
        using var cross = new SKPaint { IsAntialias = true, Color = StageActColor, StrokeWidth = 1.6f };
        canvas.DrawLine(stagePos.X - 10, stagePos.Y, stagePos.X + 10, stagePos.Y, cross);
        canvas.DrawLine(stagePos.X, stagePos.Y - 10, stagePos.X, stagePos.Y + 10, cross);

        // 振镜偏摆矢量
        var spot = vt.ToScreen(sim.CurSpotX, sim.CurSpotY);
        using var beam = new SKPaint
        { IsAntialias = true, Color = GalvoLineColor.WithAlpha(180), StrokeWidth = 1.4f };
        canvas.DrawLine(stagePos, spot, beam);

        // 激光落点
        using var spotFill = new SKPaint { IsAntialias = true, Style = SKPaintStyle.Fill };
        if (sim.CurLaserOn)
        {
            spotFill.Color = SpotColor.WithAlpha(60);
            canvas.DrawCircle(spot, 10, spotFill);
        }
        spotFill.Color = sim.CurLaserOn ? SpotColor : new SKColor(0x99, 0x99, 0x99);
        canvas.DrawCircle(spot, 4, spotFill);
    }

    private static void DrawLegend(SKCanvas canvas, float width)
    {
        using var font = new SKFont(SKTypeface.FromFamilyName("Microsoft YaHei"), 13);
        using var text = new SKPaint { IsAntialias = true };
        (SKColor color, string label)[] items =
        {
            (GeomColor, "原始图形"),
            (DrillPointColor, "钻孔位置"),
            (DrillTrajColor, "钻孔顺序轨迹"),
            (StageColor, "平台指令路径(低频)"),
            (StageActColor, "平台实际轨迹"),
            (SpotColor, "激光落点(出光)"),
            (FovColor, "振镜视场"),
            (GalvoLineColor, "振镜偏摆矢量"),
        };
        float x = 50, y = 60;
        foreach (var (color, label) in items)
        {
            using var sw = new SKPaint { Color = color, StrokeWidth = 4, IsAntialias = true };
            canvas.DrawLine(x, y - 4, x + 22, y - 4, sw);
            text.Color = new SKColor(0xCC, 0xCC, 0xD4);
            canvas.DrawText(label, x + 28, y, font, text);
            y += 20;
        }
    }

    // ================= 误差曲线 =================

    public static void DrawErrorChart(SKCanvas canvas, MainViewModel vm, float width, float height)
    {
        canvas.Clear(new SKColor(0x12, 0x12, 0x16));
        using var font = new SKFont(SKTypeface.FromFamilyName("Microsoft YaHei"), 12);
        using var text = new SKPaint { IsAntialias = true, Color = new SKColor(0xAA, 0xAA, 0xB4) };

        var sim = vm.Sim;
        if (sim == null || sim.Index < 2)
        {
            canvas.DrawText("误差监控：开始仿真后显示 平台跟随误差 与 补偿后落点误差", 16, height / 2, font, text);
            return;
        }

        int n = sim.Index;
        int window = (int)(vm.SampleRate * 5);          // 最近 5 秒
        int start = Math.Max(0, n - window);
        int count = n - start;

        // 纵轴范围
        double maxVal = 0.01;
        for (int i = start; i < n; i++)
        {
            double se = Math.Sqrt(sim.StageErrX[i] * sim.StageErrX[i] + sim.StageErrY[i] * sim.StageErrY[i]);
            if (se > maxVal) maxVal = se;
            if (sim.SpotError[i] > maxVal) maxVal = sim.SpotError[i];
        }
        maxVal *= 1.15;

        float plotL = 60, plotR = width - 12, plotT = 8, plotB = height - 20;

        using var grid = new SKPaint { Color = new SKColor(0x2A, 0x2A, 0x33), StrokeWidth = 1 };
        for (int g = 0; g <= 4; g++)
        {
            float y = plotT + (plotB - plotT) * g / 4f;
            canvas.DrawLine(plotL, y, plotR, y, grid);
            double v = maxVal * (1 - g / 4.0) * 1000;
            canvas.DrawText($"{v:F0}µm", 6, y + 4, font, text);
        }

        DrawCurve(canvas, sim, start, count, plotL, plotR, plotT, plotB, maxVal,
            i => Math.Sqrt(sim.StageErrX[i] * sim.StageErrX[i] + sim.StageErrY[i] * sim.StageErrY[i]),
            StageColor, 1.4f);
        DrawCurve(canvas, sim, start, count, plotL, plotR, plotT, plotB, maxVal,
            i => sim.SpotError[i], SpotColor, 1.6f);

        // 图例与统计
        using var sw1 = new SKPaint { Color = StageColor, StrokeWidth = 4, IsAntialias = true };
        using var sw2 = new SKPaint { Color = SpotColor, StrokeWidth = 4, IsAntialias = true };
        float lx = plotL + 10;
        canvas.DrawLine(lx, plotT + 12, lx + 20, plotT + 12, sw1);
        canvas.DrawText("平台跟随误差(未补偿)", lx + 26, plotT + 16, font, text);
        canvas.DrawLine(lx + 190, plotT + 12, lx + 210, plotT + 12, sw2);
        canvas.DrawText($"补偿后落点误差  Max {sim.MaxSpotError * 1000:F1}µm  RMS {sim.RmsSpotError * 1000:F1}µm",
            lx + 216, plotT + 16, font, text);

        canvas.DrawText($"最近 {(count / vm.SampleRate):F1} s", plotR - 80, plotB + 15, font, text);
    }

    private static void DrawCurve(SKCanvas canvas, Core.Simulation.LinkageSimulator sim,
        int start, int count, float l, float r, float t, float b, double maxVal,
        Func<int, double> selector, SKColor color, float strokeWidth)
    {
        using var paint = new SKPaint
        { IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = strokeWidth, Color = color };
        using var path = new SKPath();
        int stride = Math.Max(1, count / 2000);
        bool first = true;
        for (int k = 0; k < count; k += stride)
        {
            int i = start + k;
            float x = l + (r - l) * k / Math.Max(count - 1, 1);
            float y = b - (float)((b - t) * Math.Min(selector(i) / maxVal, 1.0));
            if (first) { path.MoveTo(x, y); first = false; }
            else path.LineTo(x, y);
        }
        canvas.DrawPath(path, paint);
    }
}
