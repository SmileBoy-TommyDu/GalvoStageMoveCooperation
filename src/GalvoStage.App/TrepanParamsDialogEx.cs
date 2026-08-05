using System.Windows;
using System.Windows.Controls;
using GalvoStage.Core.Drilling;

namespace GalvoStage.App;

/// <summary>
/// 激光钻孔工艺参数配置对话框（增强版：支持多档位独立配置）
/// </summary>
public class TrepanParamsDialogEx : Window
{
    private readonly TrepanParams[] _params = new TrepanParams[4];
    private readonly TextBox[] _powerBoxes = new TextBox[4];
    private readonly TextBox[] _ringsBoxes = new TextBox[4];
    private readonly TextBox[] _feedBoxes = new TextBox[4];
    private readonly TextBox[] _holdBoxes = new TextBox[4];
    private readonly TextBox[] _coolBoxes = new TextBox[4];
    
    public TrepanParamsDialogEx()
    {
        Title = "激光钻孔工艺参数配置（多档位）";
        Width = 700;
        Height = 550;
        WindowStartupLocation = WindowStartupLocation.CenterScreen;
        Background = System.Windows.Media.Brushes.White;
        
        // 初始化 4 个档位的参数
        _params[0] = TrepanParams.SmallHole;
        _params[1] = TrepanParams.MediumHole;
        _params[2] = TrepanParams.LargeHole;
        _params[3] = TrepanParams.ExtraLargeHole;
        
        var mainGrid = new Grid
        {
            Margin = new Thickness(20),
            RowDefinitions =
            {
                new RowDefinition { Height = GridLength.Auto },
                new RowDefinition { Height = new GridLength(1, GridUnitType.Star) },
                new RowDefinition { Height = GridLength.Auto }
            }
        };
        
        // 标题
        var title = new TextBlock
        {
            Text = "激光钻孔工艺参数配置（4 档独立设置）",
            FontSize = 16,
            FontWeight = FontWeights.Bold,
            Margin = new Thickness(0, 0, 0, 15)
        };
        Grid.SetRow(title, 0);
        mainGrid.Children.Add(title);
        
        // 创建 4 列的 DataGrid（每档一列）
        var dataGrid = new Grid
        {
            Margin = new Thickness(0, 0, 0, 15)
        };
        
        // 5 列：参数名 + 4 个档位
        dataGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(120) });
        dataGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(130) });
        dataGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(130) });
        dataGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(130) });
        dataGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(130) });
        
        // 6 行：标题 + 5 个参数
        for (int i = 0; i < 6; i++)
        {
            dataGrid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        }
        
        // 表头
        AddHeader(dataGrid, 0, 0, "参数");
        AddHeader(dataGrid, 0, 1, "微孔 (≤1mm)");
        AddHeader(dataGrid, 0, 2, "小孔 (1-3mm)");
        AddHeader(dataGrid, 0, 3, "大孔 (3-5mm)");
        AddHeader(dataGrid, 0, 4, "特大孔 (>5mm)");
        
        // 参数行
        AddParameterRow(dataGrid, 1, "激光功率 (W):", 0, _powerBoxes);
        AddParameterRow(dataGrid, 2, "补偿圈数:", 1, _ringsBoxes);
        AddParameterRow(dataGrid, 3, "进给速度 (mm/s):", 2, _feedBoxes);
        AddParameterRow(dataGrid, 4, "持留时间 (ms):", 3, _holdBoxes);
        AddParameterRow(dataGrid, 5, "冷却间隔 (ms):", 4, _coolBoxes);
        
        Grid.SetRow(dataGrid, 1);
        mainGrid.Children.Add(dataGrid);
        
        // 按钮
        var buttonPanel = new StackPanel
        {
            Orientation = Orientation.Horizontal,
            HorizontalAlignment = HorizontalAlignment.Right,
            Margin = new Thickness(0, 15, 0, 0)
        };
        
        var okButton = new Button
        {
            Content = "确定",
            Width = 80,
            Height = 30,
            Margin = new Thickness(0, 0, 10, 0),
            IsDefault = true
        };
        okButton.Click += OnOkClick;
        
        var cancelButton = new Button
        {
            Content = "取消",
            Width = 80,
            Height = 30,
            IsCancel = true
        };
        
        var resetButton = new Button
        {
            Content = "恢复默认",
            Width = 100,
            Height = 30,
            Margin = new Thickness(0, 0, 10, 0)
        };
        resetButton.Click += OnResetClick;
        
        buttonPanel.Children.Add(resetButton);
        buttonPanel.Children.Add(okButton);
        buttonPanel.Children.Add(cancelButton);
        
        Grid.SetRow(buttonPanel, 2);
        mainGrid.Children.Add(buttonPanel);
        
        Content = mainGrid;
    }
    
    private void AddHeader(Grid grid, int row, int col, string text)
    {
        var block = new TextBlock
        {
            Text = text,
            FontWeight = FontWeights.Bold,
            Margin = new Thickness(5, 5, 5, 10),
            VerticalAlignment = VerticalAlignment.Center
        };
        Grid.SetRow(block, row);
        Grid.SetColumn(block, col);
        grid.Children.Add(block);
    }
    
    private void AddParameterRow(Grid grid, int row, string label, int paramIndex, TextBox[] textBoxes)
    {
        var labelBlock = new TextBlock
        {
            Text = label,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(5, 5, 5, 10)
        };
        Grid.SetRow(labelBlock, row);
        Grid.SetColumn(labelBlock, 0);
        grid.Children.Add(labelBlock);
        
        for (int col = 1; col <= 4; col++)
        {
            var textBox = new TextBox
            {
                VerticalContentAlignment = VerticalAlignment.Center,
                Margin = new Thickness(5, 5, 5, 10)
            };
            
            // 根据参数类型填充默认值
            switch (paramIndex)
            {
                case 0: textBox.Text = _params[col - 1].Power.ToString("F0"); break;
                case 1: textBox.Text = _params[col - 1].OffsetRings.ToString(); break;
                case 2: textBox.Text = _params[col - 1].FeedRate.ToString("F0"); break;
                case 3: textBox.Text = _params[col - 1].HoldTime.ToString("F0"); break;
                case 4: textBox.Text = _params[col - 1].CoolDownInterval.ToString("F0"); break;
            }
            
            textBoxes[col - 1] = textBox;
            Grid.SetRow(textBox, row);
            Grid.SetColumn(textBox, col);
            grid.Children.Add(textBox);
        }
    }
    
    private void OnOkClick(object sender, RoutedEventArgs e)
    {
        // 验证并保存所有档位的参数
        for (int i = 0; i < 4; i++)
        {
            if (!double.TryParse(_powerBoxes[i].Text, out double power) ||
                !int.TryParse(_ringsBoxes[i].Text, out int rings) ||
                !double.TryParse(_feedBoxes[i].Text, out double feed) ||
                !double.TryParse(_holdBoxes[i].Text, out double hold) ||
                !double.TryParse(_coolBoxes[i].Text, out double cool))
            {
                MessageBox.Show($"档位 {i + 1} 的输入无效，请检查！", "错误", 
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            
            _params[i] = TrepanParams.Custom(power, rings, feed, hold, cool);
        }
        
        DialogResult = true;
        Close();
    }
    
    private void OnResetClick(object sender, RoutedEventArgs e)
    {
        // 恢复默认值
        _params[0] = TrepanParams.SmallHole;
        _params[1] = TrepanParams.MediumHole;
        _params[2] = TrepanParams.LargeHole;
        _params[3] = TrepanParams.ExtraLargeHole;
        
        for (int i = 0; i < 4; i++)
        {
            _powerBoxes[i].Text = _params[i].Power.ToString("F0");
            _ringsBoxes[i].Text = _params[i].OffsetRings.ToString();
            _feedBoxes[i].Text = _params[i].FeedRate.ToString("F0");
            _holdBoxes[i].Text = _params[i].HoldTime.ToString("F0");
            _coolBoxes[i].Text = _params[i].CoolDownInterval.ToString("F0");
        }
    }
    
    public TrepanParams[] GetResults() => _params;
}
