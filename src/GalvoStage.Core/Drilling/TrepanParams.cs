namespace GalvoStage.Core.Drilling;

/// <summary>
/// 激光钻孔环切工艺参数（按孔径分档配置）
/// </summary>
public sealed class TrepanParams
{
    /// <summary>
    /// 激光功率 (W)，范围通常为 0-15000
    /// </summary>
    public double Power { get; init; }

    /// <summary>
    /// 补偿圈数（从中心向外扩展的扩孔层数）
    /// </summary>
    public int OffsetRings { get; init; }

    /// <summary>
    /// 进给速度 (mm/s)
    /// </summary>
    public double FeedRate { get; init; }

    /// <summary>
    /// 持留时间 (ms)，用于孔边缘熔化
    /// </summary>
    public double HoldTime { get; init; }

    /// <summary>
    /// 冷却间隔 (ms)，大孔需要分层后暂停冷却
    /// </summary>
    public double CoolDownInterval { get; init; }

    /// <summary>
    /// 脉冲占空比 (0.0-1.0)，1.0 为连续输出
    /// </summary>
    public double DutyCycle { get; init; }

    /// <summary>
    /// 小孔策略：防过烧，低功率 + 少圈数
    /// </summary>
    public static readonly TrepanParams SmallHole = new()
    {
        Power = 5000,
        OffsetRings = 1,
        FeedRate = 80,
        HoldTime = 20,
        CoolDownInterval = 0,
        DutyCycle = 1.0
    };

    /// <summary>
    /// 中孔策略：均衡性能
    /// </summary>
    public static readonly TrepanParams MediumHole = new()
    {
        Power = 8000,
        OffsetRings = 2,
        FeedRate = 100,
        HoldTime = 30,
        CoolDownInterval = 50,
        DutyCycle = 1.0
    };

    /// <summary>
    /// 大孔策略：高功率 + 多层螺旋
    /// </summary>
    public static readonly TrepanParams LargeHole = new()
    {
        Power = 12000,
        OffsetRings = 3,
        FeedRate = 120,
        HoldTime = 40,
        CoolDownInterval = 100,
        DutyCycle = 0.9
    };

    /// <summary>
    /// 特大孔策略：超高功率 + 多层冷却循环
    /// </summary>
    public static readonly TrepanParams ExtraLargeHole = new()
    {
        Power = 15000,
        OffsetRings = 5,
        FeedRate = 150,
        HoldTime = 50,
        CoolDownInterval = 150,
        DutyCycle = 0.85
    };

    /// <summary>
    /// 根据孔径自动选择工艺参数分档
    /// </summary>
    public static TrepanParams CreateForDiameter(double diameter)
    {
        if (diameter <= 1.0)          // ≤1mm 微孔
            return SmallHole;
        else if (diameter <= 3.0)     // 1-3mm 中孔
            return MediumHole;
        else if (diameter <= 5.0)     // 3-5mm 大孔
            return LargeHole;
        else                          // >5mm 特大孔
            return ExtraLargeHole;
    }

    /// <summary>
    /// 自定义工艺参数（用于手动配置）
    /// </summary>
    public static TrepanParams Custom(double power, int rings, double feed, 
        double hold, double coolDown = 0, double duty = 1.0)
    {
        return new TrepanParams
        {
            Power = power,
            OffsetRings = rings,
            FeedRate = feed,
            HoldTime = hold,
            CoolDownInterval = coolDown,
            DutyCycle = duty
        };
    }

    public override string ToString()
    {
        return $"Power={Power}W, Rings={OffsetRings}, Feed={FeedRate}mm/s, Hold={HoldTime}ms";
    }
    
    /// <summary>
    /// 获取该档位的可视化颜色索引（用于仿真动画）
    /// 0=青色 (微孔), 1=绿色 (小孔), 2=黄色 (中孔), 3=橙色 (大孔), 4=红色 (特大孔)
    /// </summary>
    public int GetColorIndex()
    {
        if (Power <= 5000) return 0;         // 微孔：青色
        else if (Power <= 8000) return 1;    // 小孔：绿色
        else if (Power <= 12000) return 2;   // 中孔：黄色
        else if (Power <= 15000) return 3;   // 大孔：橙色
        else return 4;                       // 特大孔：红色
    }
}
