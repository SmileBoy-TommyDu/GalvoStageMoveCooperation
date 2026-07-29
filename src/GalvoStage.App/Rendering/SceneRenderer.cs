using System;
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

    public static void DrawScene(SKCanvas canvas, MainViewModel vm, ViewTransform vt, float width, float height)
    {
        canvas.Clear(BgColor);
        DrawGrid(canvas, vt, width, height);

        using var paint = new SKPaint { IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = 1f };

        // 1) 原始 DXF 图形
        paint.Color = GeomColor;
        paint.StrokeWidth = 1f;
        foreach (var pl in vm.Polylines)
            DrawPolyline(canvas, paint, vt, pl);

        var plan = vm.Plan;
        var sim = vm.Sim;

        // 2) 平台低频路径（分解结果）
        if (plan != null)
        {
            paint.Color = StageColor.WithAlpha(160);
            paint.StrokeWidth = 1.6f;
            DrawSampled(canvas, paint, vt, plan.StageX, plan.StageY, 0, plan.Count);
        }

        // 3) 实时加工轨迹
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

        DrawLegend(canvas, width);
    }

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
        // 坐标原点
        using var axis = new SKPaint { Color = GridColor.WithAlpha(255), StrokeWidth = 1.5f };
        var o = vt.ToScreen(0, 0);
        canvas.DrawLine(o.X - 12, o.Y, o.X + 12, o.Y, axis);
        canvas.DrawLine(o.X, o.Y - 12, o.X, o.Y + 12, axis);
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
            (StageColor, "平台指令路径(低频)"),
            (StageActColor, "平台实际轨迹"),
            (SpotColor, "激光落点(出光)"),
            (FovColor, "振镜视场"),
            (GalvoLineColor, "振镜偏摆矢量"),
        };
        float x = 16, y = 24;
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
