namespace GalvoStage.Core.Drilling;

/// <summary>
/// 材料类型枚举
/// </summary>
public enum MaterialType
{
    /// <summary>FR-4（标准 PCB 材料）</summary>
    FR4,
    
    /// <summary>铝基板</summary>
    Aluminum,
    
    /// <summary>铜基板</summary>
    Copper,
    
    /// <summary>陶瓷基板</summary>
    Ceramic,
    
    /// <summary>聚酰亚胺（柔性 PCB）</summary>
    Polyimide,
    
    /// <summary>亚克力（有机玻璃）</summary>
    Acrylic,
    
    /// <summary>自定义材料</summary>
    Custom
}

/// <summary>
/// 材料工艺库管理器
/// 根据不同材料和厚度自动推荐工艺参数
/// </summary>
public static class MaterialProcessLibrary
{
    /// <summary>
    /// 材料工艺参数配置
    /// </summary>
    public sealed class MaterialConfig
    {
        public MaterialType Material { get; init; }
        public string Name { get; init; } = "";
        public double Thickness { get; init; }  // 材料厚度 (mm)
        
        /// <summary>微孔参数（≤1mm）</summary>
        public TrepanParams MicroHoleParams { get; init; } = TrepanParams.SmallHole;
        
        /// <summary>小孔参数（1-3mm）</summary>
        public TrepanParams SmallHoleParams { get; init; } = TrepanParams.MediumHole;
        
        /// <summary>大孔参数（3-5mm）</summary>
        public TrepanParams LargeHoleParams { get; init; } = TrepanParams.LargeHole;
        
        /// <summary>特大孔参数（>5mm）</summary>
        public TrepanParams ExtraLargeHoleParams { get; init; } = TrepanParams.ExtraLargeHole;
        
        public override string ToString() => $"{Name} ({Thickness}mm)";
    }
    
    /// <summary>
    /// 预设材料工艺库
    /// </summary>
    private static readonly Dictionary<MaterialType, List<MaterialConfig>> _library = new()
    {
        // FR-4（标准 PCB）
        [MaterialType.FR4] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.FR4,
                Name = "FR-4 薄板",
                Thickness = 0.8,
                MicroHoleParams = TrepanParams.Custom(4000, 1, 60, 15),
                SmallHoleParams = TrepanParams.Custom(6000, 2, 80, 20),
                LargeHoleParams = TrepanParams.Custom(9000, 2, 100, 30),
                ExtraLargeHoleParams = TrepanParams.Custom(12000, 3, 120, 40)
            },
            new MaterialConfig
            {
                Material = MaterialType.FR4,
                Name = "FR-4 标准板",
                Thickness = 1.6,
                MicroHoleParams = TrepanParams.Custom(5000, 1, 70, 20),
                SmallHoleParams = TrepanParams.Custom(7000, 2, 90, 25),
                LargeHoleParams = TrepanParams.Custom(10000, 3, 110, 35),
                ExtraLargeHoleParams = TrepanParams.Custom(13000, 4, 130, 50)
            },
            new MaterialConfig
            {
                Material = MaterialType.FR4,
                Name = "FR-4 厚板",
                Thickness = 2.4,
                MicroHoleParams = TrepanParams.Custom(6000, 2, 80, 25),
                SmallHoleParams = TrepanParams.Custom(8000, 2, 100, 30),
                LargeHoleParams = TrepanParams.Custom(11000, 3, 120, 40),
                ExtraLargeHoleParams = TrepanParams.Custom(14000, 5, 140, 60)
            }
        },
        
        // 铝基板
        [MaterialType.Aluminum] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.Aluminum,
                Name = "铝基板 薄",
                Thickness = 1.0,
                MicroHoleParams = TrepanParams.Custom(7000, 2, 50, 30, 100, 0.8),
                SmallHoleParams = TrepanParams.Custom(9000, 3, 60, 40, 120, 0.75),
                LargeHoleParams = TrepanParams.Custom(12000, 4, 70, 50, 150, 0.7),
                ExtraLargeHoleParams = TrepanParams.Custom(15000, 6, 80, 60, 180, 0.65)
            },
            new MaterialConfig
            {
                Material = MaterialType.Aluminum,
                Name = "铝基板 标准",
                Thickness = 2.0,
                MicroHoleParams = TrepanParams.Custom(8000, 2, 55, 35, 120, 0.75),
                SmallHoleParams = TrepanParams.Custom(10000, 3, 65, 45, 140, 0.7),
                LargeHoleParams = TrepanParams.Custom(13000, 5, 75, 55, 170, 0.65),
                ExtraLargeHoleParams = TrepanParams.Custom(15000, 7, 85, 65, 200, 0.6)
            }
        },
        
        // 铜基板
        [MaterialType.Copper] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.Copper,
                Name = "铜基板 薄",
                Thickness = 0.5,
                MicroHoleParams = TrepanParams.Custom(8000, 2, 40, 40, 150, 0.7),
                SmallHoleParams = TrepanParams.Custom(10000, 3, 50, 50, 180, 0.65),
                LargeHoleParams = TrepanParams.Custom(13000, 4, 60, 60, 200, 0.6),
                ExtraLargeHoleParams = TrepanParams.Custom(15000, 6, 70, 70, 220, 0.55)
            }
        },
        
        // 陶瓷基板
        [MaterialType.Ceramic] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.Ceramic,
                Name = "陶瓷基板 标准",
                Thickness = 0.6,
                MicroHoleParams = TrepanParams.Custom(9000, 3, 30, 50, 200, 0.6),
                SmallHoleParams = TrepanParams.Custom(11000, 4, 40, 60, 220, 0.55),
                LargeHoleParams = TrepanParams.Custom(14000, 5, 50, 70, 250, 0.5),
                ExtraLargeHoleParams = TrepanParams.Custom(15000, 8, 60, 80, 280, 0.45)
            }
        },
        
        // 聚酰亚胺（柔性 PCB）
        [MaterialType.Polyimide] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.Polyimide,
                Name = "聚酰亚胺 柔性板",
                Thickness = 0.2,
                MicroHoleParams = TrepanParams.Custom(3000, 1, 100, 10),
                SmallHoleParams = TrepanParams.Custom(5000, 1, 120, 15),
                LargeHoleParams = TrepanParams.Custom(7000, 2, 140, 20),
                ExtraLargeHoleParams = TrepanParams.Custom(9000, 3, 160, 25)
            }
        },
        
        // 亚克力
        [MaterialType.Acrylic] = new List<MaterialConfig>
        {
            new MaterialConfig
            {
                Material = MaterialType.Acrylic,
                Name = "亚克力 薄",
                Thickness = 2.0,
                MicroHoleParams = TrepanParams.Custom(5000, 1, 150, 15),
                SmallHoleParams = TrepanParams.Custom(7000, 2, 180, 20),
                LargeHoleParams = TrepanParams.Custom(9000, 2, 200, 25),
                ExtraLargeHoleParams = TrepanParams.Custom(11000, 3, 220, 30)
            },
            new MaterialConfig
            {
                Material = MaterialType.Acrylic,
                Name = "亚克力 厚",
                Thickness = 5.0,
                MicroHoleParams = TrepanParams.Custom(6000, 2, 120, 25),
                SmallHoleParams = TrepanParams.Custom(8000, 3, 150, 30),
                LargeHoleParams = TrepanParams.Custom(10000, 4, 180, 35),
                ExtraLargeHoleParams = TrepanParams.Custom(12000, 5, 200, 40)
            }
        }
    };
    
    /// <summary>
    /// 根据材料和厚度获取推荐工艺参数
    /// </summary>
    public static MaterialConfig GetRecommendedConfig(
        MaterialType material, double thickness)
    {
        if (!_library.ContainsKey(material))
            return GetDefaultConfig(material, thickness);
        
        var configs = _library[material];
        
        // 找到最接近的厚度
        MaterialConfig bestMatch = configs[0];
        double minDiff = Math.Abs(configs[0].Thickness - thickness);
        
        for (int i = 1; i < configs.Count; i++)
        {
            double diff = Math.Abs(configs[i].Thickness - thickness);
            if (diff < minDiff)
            {
                minDiff = diff;
                bestMatch = configs[i];
            }
        }
        
        return bestMatch;
    }
    
    /// <summary>
    /// 根据孔径获取推荐参数（简化版）
    /// </summary>
    public static TrepanParams GetParamsForHole(
        MaterialType material, double thickness, double diameter)
    {
        var config = GetRecommendedConfig(material, thickness);
        
        if (diameter <= 1.0) return config.MicroHoleParams;
        else if (diameter <= 3.0) return config.SmallHoleParams;
        else if (diameter <= 5.0) return config.LargeHoleParams;
        else return config.ExtraLargeHoleParams;
    }
    
    /// <summary>
    /// 获取默认配置（未知材料时）
    /// </summary>
    private static MaterialConfig GetDefaultConfig(MaterialType material, double thickness)
    {
        return new MaterialConfig
        {
            Material = material,
            Name = $"{material} 默认",
            Thickness = thickness,
            MicroHoleParams = TrepanParams.SmallHole,
            SmallHoleParams = TrepanParams.MediumHole,
            LargeHoleParams = TrepanParams.LargeHole,
            ExtraLargeHoleParams = TrepanParams.ExtraLargeHole
        };
    }
    
    /// <summary>
    /// 添加自定义材料配置
    /// </summary>
    public static void AddCustomConfig(MaterialConfig config)
    {
        if (!_library.ContainsKey(MaterialType.Custom))
            _library[MaterialType.Custom] = new List<MaterialConfig>();
        
        _library[MaterialType.Custom].Add(config);
    }
    
    /// <summary>
    /// 获取所有可用材料列表
    /// </summary>
    public static IReadOnlyList<MaterialType> GetAvailableMaterials()
    {
        return _library.Keys.ToList();
    }
    
    /// <summary>
    /// 获取指定材料的所有厚度配置
    /// </summary>
    public static IReadOnlyList<MaterialConfig> GetConfigsForMaterial(MaterialType material)
    {
        if (_library.ContainsKey(material))
            return _library[material].AsReadOnly();
        return new List<MaterialConfig>().AsReadOnly();
    }
}
