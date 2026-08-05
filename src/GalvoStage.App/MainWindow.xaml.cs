using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using GalvoStage.App.Rendering;
using GalvoStage.App.ViewModels;
using Microsoft.Win32;
using SkiaSharp.Views.Desktop;
using SkiaSharp.Views.WPF;

namespace GalvoStage.App;

/// <summary>布尔取反转换器（用于"自动截止频率"时禁用手动滑条）</summary>
public sealed class InverseBoolConverter : IValueConverter
{
    public static readonly InverseBoolConverter Instance = new();
    public object Convert(object value, Type t, object p, CultureInfo c) => value is bool b && !b;
    public object ConvertBack(object value, Type t, object p, CultureInfo c) => value is bool b && !b;
}

public partial class MainWindow : Window
{
    private readonly MainViewModel _vm = new();
    private readonly ViewTransform _vt = new();          // 平台视图（世界坐标）
    private readonly ViewTransform _galvoVt = new();     // 振镜视图（局部坐标 ±FOV）
    private readonly Stopwatch _clock = new();
    private double _lastTime;
    private bool _fitPending = true;
    private bool _originInitialized;           // 首帧将坐标原点置于画布中心
    private bool _galvoFitPending = true;
    private bool _galvoOriginInitialized;
    private (double X, double Y)? _mouseWorld; // 平台视图鼠标世界坐标
    private (double X, double Y)? _galvoMouseWorld; // 振镜视图鼠标坐标

    private bool _panning;
    private Point _panStart;
    private bool _galvoPanning;
    private Point _galvoPanStart;

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;
        _vm.SceneChanged += () => { _galvoFitPending = true; InvalidateCanvases(); };
        _vm.PropertyChanged += (_, e) =>
        {
            if (e.PropertyName == nameof(MainViewModel.IsRunning))
                BtnStart.Content = _vm.IsRunning ? "⏸ 暂停" : "▶ 开始仿真";
        };

        _clock.Start();
        CompositionTarget.Rendering += OnFrame;
    }

    // ================= 渲染循环 =================

    private void OnFrame(object? sender, EventArgs e)
    {
        double now = _clock.Elapsed.TotalSeconds;
        double dt = Math.Min(now - _lastTime, 0.1);
        _lastTime = now;

        if (_vm.IsRunning)
        {
            _vm.Advance(dt);
            InvalidateCanvases();
        }
    }

    private void InvalidateCanvases()
    {
        StageCanvas.InvalidateVisual();
        GalvoCanvas.InvalidateVisual();
        ChartCanvas.InvalidateVisual();
    }

    private void OnPaintStage(object? sender, SKPaintSurfaceEventArgs e)
    {
        float w = e.Info.Width, h = e.Info.Height;
        // 初始化：空画布也显示 XY 轴，原点居中
        if (!_originInitialized && w > 0 && h > 0)
        {
            _vt.CenterOrigin(w, h);
            _originInitialized = true;
        }
        if (_fitPending && (_vm.Polylines.Count > 0 || _vm.DrillingPattern?.Holes.Count > 0))
        {
            FitView(w, h);
            _fitPending = false;
        }
        SceneRenderer.DrawScene(e.Surface.Canvas, _vm, _vt, w, h, _mouseWorld);
    }

    private void OnPaintGalvo(object? sender, SKPaintSurfaceEventArgs e)
    {
        float w = e.Info.Width, h = e.Info.Height;
        if (!_galvoOriginInitialized && w > 0 && h > 0)
        {
            _galvoVt.CenterOrigin(w, h);
            _galvoOriginInitialized = true;
        }
        if (_galvoFitPending && w > 0 && h > 0)
        {
            FitGalvoView(w, h);
            _galvoFitPending = false;
        }
        SceneRenderer.DrawGalvoView(e.Surface.Canvas, _vm, _galvoVt, w, h, _galvoMouseWorld);
    }

    /// <summary>振镜视图自适应：以振镜中心为原点，铺满 ±FOV 视场</summary>
    private void FitGalvoView(float w, float h)
    {
        double m = Math.Max(_vm.GalvoFov * 1.3, 1);
        _galvoVt.Fit(-m, -m, m, m, w, h);
    }

    private void OnPaintChart(object? sender, SKPaintSurfaceEventArgs e)
        => SceneRenderer.DrawErrorChart(e.Surface.Canvas, _vm, e.Info.Width, e.Info.Height);

    private void FitView(float w, float h)
    {
        // 使用导入时构建的全局包围盒缓存，O(1)，避免全量扫描顶点
        double minX, minY, maxX, maxY;
        var gc = _vm.GeometryCache;
        if (gc != null && gc.HasBounds)
        {
            minX = gc.WorldMinX; minY = gc.WorldMinY;
            maxX = gc.WorldMaxX; maxY = gc.WorldMaxY;
        }
        else if (_vm.DrillingPattern?.Bounds is { } db)
        {
            minX = db.MinX; minY = db.MinY;
            maxX = db.MaxX; maxY = db.MaxY;
        }
        else { minX = -50; maxX = 50; minY = -50; maxY = 50; }
        // 预留视场余量
        double m = Math.Max(_vm.GalvoFov * 1.5, 5);
        _vt.Fit(minX - m, minY - m, maxX + m, maxY + m, w, h);
    }

    // ================= 按钮 =================

    private void OnImportClick(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Filter = "DXF 图形文件 (*.dxf)|*.dxf|所有文件 (*.*)|*.*",
            Title = "导入 DXF 加工图形"
        };
        if (dlg.ShowDialog(this) != true) return;
        try
        {
            _vm.ImportDxf(dlg.FileName);

            _fitPending = true;
            InvalidateCanvases();
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, $"DXF 解析失败：{ex.Message}", "导入错误", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void OnLoadDemoClick(object sender, RoutedEventArgs e)
    {
        string path = Path.Combine(AppContext.BaseDirectory, "Samples", "demo.dxf");
        if (!File.Exists(path))
        {
            MessageBox.Show(this, $"示例文件不存在：{path}", "提示", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        _vm.ImportDxf(path);
        _fitPending = true;
        InvalidateCanvases();
    }

    private void OnDecomposeClick(object sender, RoutedEventArgs e)
    {
        // 钻孔模式：已规划钻孔轨迹时走钻孔仿真准备分支
        if (_vm.Polylines.Count == 0 && _vm.DrillingTrajectory != null)
        {
            Mouse.OverrideCursor = Cursors.Wait;
            try { _vm.DecomposeDrilling(); }
            finally { Mouse.OverrideCursor = null; }
            InvalidateCanvases();
            return;
        }
        if (_vm.Polylines.Count == 0)
        {
            MessageBox.Show(this, "请先导入 DXF 图形，或导入钻孔 DXF 并完成路径规划。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        Mouse.OverrideCursor = Cursors.Wait;
        try { _vm.Decompose(); }
        finally { Mouse.OverrideCursor = null; }
        InvalidateCanvases();
    }

    /// <summary>导入混合 DXF：一次解析同时提取折线（轮廓）与钻孔（CIRCLE），双模式分离加工</summary>
    private void OnImportMixedClick(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Filter = "DXF 混合文件 (*.dxf)|*.dxf|所有文件 (*.*)|*.*",
            Title = "导入混合 DXF（同时提取轮廓 + 钻孔）"
        };
        if (dlg.ShowDialog(this) != true) return;
        try
        {
            _vm.ImportMixed(dlg.FileName);
            _fitPending = true;
            InvalidateCanvases();

            // 提示用户是否立即执行双模式分解
            string summary = $"轮廓：{_vm.Polylines.Count:N0} 条\n钻孔：{_vm.DrillingPattern?.Holes.Count ?? 0:N0} 个";
            var result = MessageBox.Show(
                this,
                $"已解析混合特征：\n{summary}\n\n是否立即执行双模式分解？\n（折线链路 + 钻孔链路独立规划）",
                "双模式分离加工",
                MessageBoxButton.YesNo,
                MessageBoxImage.Question);

            if (result == MessageBoxResult.Yes)
            {
                Mouse.OverrideCursor = Cursors.Wait;
                try { _vm.DecomposeBoth(); }
                finally { Mouse.OverrideCursor = null; }
                InvalidateCanvases();
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, $"DXF 混合解析失败：{ex.Message}", "导入错误", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    /// <summary>双模式分解：同时触发折线链路（频域分解）与钻孔链路（振镜优先聚类）</summary>
    private void OnDecomposeBothClick(object sender, RoutedEventArgs e)
    {
        if (_vm.Polylines.Count == 0 && (_vm.DrillingPattern == null || _vm.DrillingPattern.Holes.Count == 0))
        {
            MessageBox.Show(this, "请先导入混合 DXF（同时包含轮廓与钻孔）。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        Mouse.OverrideCursor = Cursors.Wait;
        try { _vm.DecomposeBoth(); }
        finally { Mouse.OverrideCursor = null; }
        InvalidateCanvases();
    }
    
    /// <summary>导出激光钻孔 G 代码（使用环切工艺参数）</summary>
    private void OnExportLaserGCodeClick(object sender, RoutedEventArgs e)
    {
        if (_vm.DrillingTrajectory == null || _vm.DrillingTrajectory.Moves.Count == 0)
        {
            MessageBox.Show(this, "请先执行双模式分解完成钻孔路径规划。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        
        var dlg = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "G 代码文件 (*.nc;*.gcode)|*.nc;*.gcode|所有文件 (*.*)|*.*",
            Title = "导出激光钻孔 G 代码（环切工艺）",
            FileName = "laser_drilling.nc"
        };
        
        if (dlg.ShowDialog() == true)
        {
            try
            {
                int count = GalvoStage.Core.Drilling.GCodeExporter.ExportLaserDrilling(
                    _vm.DrillingTrajectory, dlg.FileName);
                MessageBox.Show(this, 
                    $"已成功导出 {count} 个孔的激光钻孔 G 代码\n文件：{dlg.FileName}", 
                    "导出成功", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, $"导出失败：{ex.Message}", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
    }
    
    /// <summary>打开工艺参数配置对话框</summary>
    private void OnConfigTrepanParamsClick(object sender, RoutedEventArgs e)
    {
        var dialog = new TrepanParamsDialog(
            GalvoStage.Core.Drilling.TrepanParams.MediumHole, 
            "中孔 (1-3mm)");
        
        if (dialog.ShowDialog() == true)
        {
            var result = dialog.GetResult();
            MessageBox.Show(this, 
                $"工艺参数已更新：\n{result}", 
                "配置成功", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void OnStartPauseClick(object sender, RoutedEventArgs e)
    {
        if (_vm.Plan == null)
        {
            MessageBox.Show(this, "请先执行路径分解。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        if (_vm.Sim != null && _vm.Sim.Done) _vm.RebuildSimulator();
        _vm.IsRunning = !_vm.IsRunning;
    }

    private void OnResetClick(object sender, RoutedEventArgs e)
    {
        _vm.RebuildSimulator();
        InvalidateCanvases();
    }

    /// <summary>导入 PCB 钻孔 DXF（CIRCLE 圆心）</summary>
    private async void OnImportDrillClick(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Filter = "PCB 钻孔 DXF 文件 (*.dxf)|*.dxf|所有文件 (*.*)|*.*",
            Title = "导入钻孔 DXF（包含 CIRCLE/POINT 实体）"
        };
        if (dlg.ShowDialog(this) != true) return;
        
        try
        {
            _vm.ImportDrillingFile(dlg.FileName);
            _fitPending = true;
            InvalidateCanvases();
            
            // 提示用户是否立即规划路径
            var result = MessageBox.Show(
                this, 
                $"已导入 {_vm.DrillingPattern?.Holes.Count:N0} 个孔。\n\n是否立即开始路径规划？\n\n⚠️ 注意：超大文件可能需要数分钟。",
                "确认路径规划",
                MessageBoxButton.YesNo,
                MessageBoxImage.Question);
            
            if (result == MessageBoxResult.Yes)
            {
                Mouse.OverrideCursor = Cursors.Wait;
                try { await _vm.PlanDrillingPathAsync(); }
                finally { Mouse.OverrideCursor = null; }
                InvalidateCanvases();
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, $"DXF 解析失败：{ex.Message}", "导入错误", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }
    
    private async void OnPlanDrillClick(object sender, RoutedEventArgs e)
    {
        if (_vm.DrillingPattern == null)
        {
            MessageBox.Show(this, "请先导入钻孔 DXF。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        
        Mouse.OverrideCursor = Cursors.Wait;
        try
        {
            await _vm.PlanDrillingPathAsync();
        }
        finally
        {
            Mouse.OverrideCursor = null;
        }
        InvalidateCanvases();
    }

    /// <summary>导出已规划钻孔轨迹为 G 代码（按孔径分组换刀）</summary>
    private void OnExportGCodeClick(object sender, RoutedEventArgs e)
    {
        if (_vm.DrillingTrajectory == null || _vm.DrillingTrajectory.Moves.Count == 0)
        {
            MessageBox.Show(this, "请先完成钻孔路径规划。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        
        var dlg = new SaveFileDialog
        {
            Filter = "G 代码文件 (*.nc)|*.nc|G 代码文件 (*.gcode)|*.gcode|所有文件 (*.*)|*.*",
            Title = "导出钻孔 G 代码",
            FileName = "drill_program.nc"
        };
        if (dlg.ShowDialog(this) != true) return;
        
        Mouse.OverrideCursor = Cursors.Wait;
        try
        {
            if (_vm.ExportGCode(dlg.FileName))
                MessageBox.Show(this, $"G 代码已导出：\n{dlg.FileName}", "导出成功", MessageBoxButton.OK, MessageBoxImage.Information);
        }
        finally
        {
            Mouse.OverrideCursor = null;
        }
    }

    // ================= 视图交互 =================

    private static void ZoomCanvas(SKElement c, ViewTransform vt, MouseWheelEventArgs e)
    {
        var pos = e.GetPosition(c);
        var dpi = VisualTreeHelper.GetDpi(c);
        vt.ZoomAt((float)(pos.X * dpi.DpiScaleX), (float)(pos.Y * dpi.DpiScaleY),
            e.Delta > 0 ? 1.15 : 1 / 1.15);
        c.InvalidateVisual();
    }

    // ---- 平台视图（世界坐标）----
    private void OnStageWheel(object sender, MouseWheelEventArgs e) => ZoomCanvas(StageCanvas, _vt, e);

    private void OnStageMouseDown(object sender, MouseButtonEventArgs e)
    {
        if (e.ChangedButton == MouseButton.Left && e.ClickCount == 2)
        {
            var dpi = VisualTreeHelper.GetDpi(StageCanvas);
            FitView((float)(StageCanvas.ActualWidth * dpi.DpiScaleX),
                    (float)(StageCanvas.ActualHeight * dpi.DpiScaleY));
            StageCanvas.InvalidateVisual();
            return;
        }
        if (e.ChangedButton is MouseButton.Left or MouseButton.Middle)
        {
            _panning = true;
            _panStart = e.GetPosition(StageCanvas);
            StageCanvas.CaptureMouse();
        }
    }

    private void OnStageMouseMove(object sender, MouseEventArgs e)
    {
        var pos = e.GetPosition(StageCanvas);
        var dpi = VisualTreeHelper.GetDpi(StageCanvas);
        if (_panning)
        {
            _vt.Pan((pos.X - _panStart.X) * dpi.DpiScaleX, (pos.Y - _panStart.Y) * dpi.DpiScaleY);
            _panStart = pos;
        }
        _mouseWorld = _vt.ToWorld((float)(pos.X * dpi.DpiScaleX), (float)(pos.Y * dpi.DpiScaleY));
        StageCanvas.InvalidateVisual();
    }

    private void OnStageMouseUp(object sender, MouseButtonEventArgs e)
    {
        _panning = false;
        StageCanvas.ReleaseMouseCapture();
    }

    // ---- 振镜视图（局部坐标 ±FOV）----
    private void OnGalvoWheel(object sender, MouseWheelEventArgs e) => ZoomCanvas(GalvoCanvas, _galvoVt, e);

    private void OnGalvoMouseDown(object sender, MouseButtonEventArgs e)
    {
        if (e.ChangedButton == MouseButton.Left && e.ClickCount == 2)
        {
            var dpi = VisualTreeHelper.GetDpi(GalvoCanvas);
            FitGalvoView((float)(GalvoCanvas.ActualWidth * dpi.DpiScaleX),
                         (float)(GalvoCanvas.ActualHeight * dpi.DpiScaleY));
            GalvoCanvas.InvalidateVisual();
            return;
        }
        if (e.ChangedButton is MouseButton.Left or MouseButton.Middle)
        {
            _galvoPanning = true;
            _galvoPanStart = e.GetPosition(GalvoCanvas);
            GalvoCanvas.CaptureMouse();
        }
    }

    private void OnGalvoMouseMove(object sender, MouseEventArgs e)
    {
        var pos = e.GetPosition(GalvoCanvas);
        var dpi = VisualTreeHelper.GetDpi(GalvoCanvas);
        if (_galvoPanning)
        {
            _galvoVt.Pan((pos.X - _galvoPanStart.X) * dpi.DpiScaleX, (pos.Y - _galvoPanStart.Y) * dpi.DpiScaleY);
            _galvoPanStart = pos;
        }
        _galvoMouseWorld = _galvoVt.ToWorld((float)(pos.X * dpi.DpiScaleX), (float)(pos.Y * dpi.DpiScaleY));
        GalvoCanvas.InvalidateVisual();
    }

    private void OnGalvoMouseUp(object sender, MouseButtonEventArgs e)
    {
        _galvoPanning = false;
        GalvoCanvas.ReleaseMouseCapture();
    }
}
