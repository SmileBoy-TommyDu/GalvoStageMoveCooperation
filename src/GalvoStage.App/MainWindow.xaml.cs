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
        if (_fitPending && _vm.Polylines.Count > 0)
        {
            FitView(w, h);
            _fitPending = false;
        }
        SceneRenderer.DrawScene(e.Surface.Canvas, _vm, _vt, w, h);
    }

    private void OnPaintChart(object? sender, SKPaintSurfaceEventArgs e)
        => SceneRenderer.DrawErrorChart(e.Surface.Canvas, _vm, e.Info.Width, e.Info.Height);

    private void FitView(float w, float h)
    {
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        foreach (var pl in _vm.Polylines)
            foreach (var p in pl.Points)
            {
                if (p.X < minX) minX = p.X;
                if (p.X > maxX) maxX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.Y > maxY) maxY = p.Y;
            }
        if (minX > maxX) { minX = -50; maxX = 50; minY = -50; maxY = 50; }
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
        if (_vm.Polylines.Count == 0)
        {
            MessageBox.Show(this, "请先导入 DXF 图形。", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
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
        if (!_panning) return;
        var pos = e.GetPosition(MainCanvas);
        var dpi = VisualTreeHelper.GetDpi(MainCanvas);
        _vt.Pan((pos.X - _panStart.X) * dpi.DpiScaleX, (pos.Y - _panStart.Y) * dpi.DpiScaleY);
        _panStart = pos;
        MainCanvas.InvalidateVisual();
    }

    private void OnCanvasMouseUp(object sender, MouseButtonEventArgs e)
    {
        _panning = false;
        MainCanvas.ReleaseMouseCapture();
    }
}
