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
    private readonly ViewTransform _vt = new();
    private readonly Stopwatch _clock = new();
    private double _lastTime;
    private bool _fitPending = true;
    private bool _originInitialized;           // 首帧将坐标原点置于画布中心
    private (double X, double Y)? _mouseWorld; // 鼠标世界坐标

    private bool _panning;
    private Point _panStart;

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;
        _vm.SceneChanged += () => { InvalidateCanvases(); };
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
        MainCanvas.InvalidateVisual();
        ChartCanvas.InvalidateVisual();
    }

    private void OnPaintMain(object? sender, SKPaintSurfaceEventArgs e)
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

    private void OnCanvasWheel(object sender, MouseWheelEventArgs e)
    {
        var pos = e.GetPosition(MainCanvas);
        var dpi = VisualTreeHelper.GetDpi(MainCanvas);
        _vt.ZoomAt((float)(pos.X * dpi.DpiScaleX), (float)(pos.Y * dpi.DpiScaleY),
            e.Delta > 0 ? 1.15 : 1 / 1.15);
        MainCanvas.InvalidateVisual();
    }

    private void OnCanvasMouseDown(object sender, MouseButtonEventArgs e)
    {
        if (e.ChangedButton == MouseButton.Left && e.ClickCount == 2)
        {
            var dpi = VisualTreeHelper.GetDpi(MainCanvas);
            FitView((float)(MainCanvas.ActualWidth * dpi.DpiScaleX),
                    (float)(MainCanvas.ActualHeight * dpi.DpiScaleY));
            MainCanvas.InvalidateVisual();
            return;
        }
        if (e.ChangedButton is MouseButton.Left or MouseButton.Middle)
        {
            _panning = true;
            _panStart = e.GetPosition(MainCanvas);
            MainCanvas.CaptureMouse();
        }
    }

    private void OnCanvasMouseMove(object sender, MouseEventArgs e)
    {
        var pos = e.GetPosition(MainCanvas);
        var dpi = VisualTreeHelper.GetDpi(MainCanvas);
        
        if (!_panning) return;
        _vt.Pan((pos.X - _panStart.X) * dpi.DpiScaleX, (pos.Y - _panStart.Y) * dpi.DpiScaleY);
        _panStart = pos;
        MainCanvas.InvalidateVisual();
    }

    private void OnCanvasMouseMoveUpdatePos(object sender, MouseEventArgs e)
    {
        var pos = e.GetPosition(MainCanvas);
        var dpi = VisualTreeHelper.GetDpi(MainCanvas);
        var screenPx = new SkiaSharp.SKPoint((float)(pos.X * dpi.DpiScaleX), (float)(pos.Y * dpi.DpiScaleY));
        _mouseWorld = _vt.ToWorld(screenPx.X, screenPx.Y);
        MainCanvas.InvalidateVisual();
    }

    private void OnCanvasMouseUp(object sender, MouseButtonEventArgs e)
    {
        _panning = false;
        MainCanvas.ReleaseMouseCapture();
    }
}
