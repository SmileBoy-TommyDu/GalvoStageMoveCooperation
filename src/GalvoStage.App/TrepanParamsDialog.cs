using System.Windows;
using System.Windows.Controls;
using GalvoStage.Core.Drilling;

namespace GalvoStage.App;

/// <summary>
/// 激光钻孔工艺参数配置对话框
/// </summary>
public class TrepanParamsDialog : Window
{
    private readonly TrepanParams _params;
    private readonly string _presetName;
    
    public TrepanParamsDialog(TrepanParams currentParams, string presetName)
    {
        _params = currentParams;
        _presetName = presetName;
        
        Title = $"工艺参数配置 - {presetName}";
        Width = 450;
        Height = 380;
        WindowStartupLocation = WindowStartupLocation.CenterScreen;
        Background = System.Windows.Media.Brushes.White;
        
        var grid = new Grid
        {
            Margin = new Thickness(20),
            RowDefinitions =
            {
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = GridLength.Auto }
            },
            ColumnDefinitions =
            {
                new ColumnDefinition { Width = new GridLength(120) },
                new ColumnDefinition { Width = new GridLength(200) },
                new ColumnDefinition { Width = GridLength.Auto }
            }
        };
        
        // 标题
        var title = new TextBlock
        {
            Text = $"激光钻孔工艺参数 - {_presetName}",
            FontSize = 16,
            FontWeight = FontWeights.Bold,
            Margin = new Thickness(0, 0, 0, 20)
        };
        Grid.SetColumnSpan(title, 3);
        grid.Children.Add(title);
        
        // 功率
        AddRow(grid, 1, "激光功率 (W):", _params.Power.ToString("F0"), out var powerBox);
        // 圈数
        AddRow(grid, 2, "补偿圈数:", _params.OffsetRings.ToString(), out var ringsBox);
        // 进给
        AddRow(grid, 3, "进给速度 (mm/s):", _params.FeedRate.ToString("F0"), out var feedBox);
        // 持留
        AddRow(grid, 4, "持留时间 (ms):", _params.HoldTime.ToString("F0"), out var holdBox);
        // 冷却
        AddRow(grid, 5, "冷却间隔 (ms):", _params.CoolDownInterval.ToString("F0"), out var coolBox);
        
        // 按钮
        var buttonPanel = new StackPanel
        {
            Orientation = Orientation.Horizontal,
            HorizontalAlignment = HorizontalAlignment.Right,
            Margin = new Thickness(0, 20, 0, 0)
        };
        
        var okButton = new Button
        {
            Content = "确定",
            Width = 80,
            Height = 30,
            Margin = new Thickness(0, 0, 10, 0),
            IsDefault = true
        };
        okButton.Click += (s, e) =>
        {
            if (double.TryParse(powerBox.Text, out double power) &&
                int.TryParse(ringsBox.Text, out int rings) &&
                double.TryParse(feedBox.Text, out double feed) &&
                double.TryParse(holdBox.Text, out double hold) &&
                double.TryParse(coolBox.Text, out double cool))
            {
                DialogResult = true;
                Close();
            }
            else
            {
                MessageBox.Show("请输入有效的数值！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        };
        
        var cancelButton = new Button
        {
            Content = "取消",
            Width = 80,
            Height = 30,
            IsCancel = true
        };
        
        buttonPanel.Children.Add(okButton);
        buttonPanel.Children.Add(cancelButton);
        Grid.SetColumnSpan(buttonPanel, 3);
        grid.Children.Add(buttonPanel);
        
        Content = grid;
    }
    
    private void AddRow(Grid grid, int row, string label, string value, out TextBox textBox)
    {
        var labelBlock = new TextBlock
        {
            Text = label,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(0, 0, 10, 10)
        };
        Grid.SetRow(labelBlock, row);
        Grid.SetColumn(labelBlock, 0);
        grid.Children.Add(labelBlock);
        
        textBox = new TextBox
        {
            Text = value,
            VerticalContentAlignment = VerticalAlignment.Center,
            Margin = new Thickness(0, 0, 10, 10)
        };
        Grid.SetRow(textBox, row);
        Grid.SetColumn(textBox, 1);
        grid.Children.Add(textBox);
    }
    
    public TrepanParams GetResult()
    {
        return _params;
    }
}
