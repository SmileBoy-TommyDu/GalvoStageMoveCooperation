using System.Collections.Generic;

namespace GalvoStage.Core.Geometry.Drilling;

/// <summary>
/// PCB 钻孔点集数据模型（与 PathPolyline 并列的独立加工模式）
/// 完全独立于激光路径逻辑，只处理离散孔位
/// </summary>
public sealed class DrillingPattern
{
    /// <summary>单个钻孔点</summary>
    public struct Hole
    {
        public double X, Y;         // 位置 (mm)
        public double Diameter;     // 孔径 (mm)，0 表示未知
        public string Layer;        // DXF 图层
        public int OriginalIndex;   // 原始索引（用于调试）
        
        /// <summary>构造器：带坐标和图层信息</summary>
        public Hole(double x, double y, string layer, int index)
        {
            X = x; Y = y; Diameter = 0; Layer = layer; OriginalIndex = index;
        }

        /// <summary>构造器：带孔径</summary>
        public Hole(double x, double y, double diameter, string layer, int index)
        {
            X = x; Y = y; Diameter = diameter; Layer = layer; OriginalIndex = index;
        }
    }

    public List<Hole> Holes { get; } = new();
    
    /// <summary>全局包围盒（缓存）</summary>
    public (double MinX, double MinY, double MaxX, double MaxY)? Bounds { get; private set; }
    
    /// <summary>按图层分组的字典</summary>
    public Dictionary<string, int> LayerCounts { get; private set; } = new();

    /// <summary>按孔径分组的字典（直径 mm → 孔数，保留 3 位小数归类）</summary>
    public Dictionary<double, int> DiameterCounts { get; private set; } = new();
    
    /// <summary>重新计算包围盒和图层统计</summary>
    public void RecomputeBounds()
    {
        if (Holes.Count == 0)
        {
            Bounds = null;
            LayerCounts.Clear();
            DiameterCounts.Clear();
            return;
        }
        
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        var layerMap = new Dictionary<string, int>();
        var diaMap = new Dictionary<double, int>();
        
        for (int i = 0; i < Holes.Count; i++)
        {
            var h = Holes[i];
            if (h.X < minX) minX = h.X;
            if (h.Y < minY) minY = h.Y;
            if (h.X > maxX) maxX = h.X;
            if (h.Y > maxY) maxY = h.Y;
            
            if (layerMap.TryGetValue(h.Layer, out int lc))
                layerMap[h.Layer] = lc + 1;
            else
                layerMap[h.Layer] = 1;

            double d = System.Math.Round(h.Diameter, 3);
            if (diaMap.TryGetValue(d, out int dc))
                diaMap[d] = dc + 1;
            else
                diaMap[d] = 1;
        }
        
        Bounds = (minX, minY, maxX, maxY);
        LayerCounts = new Dictionary<string, int>(layerMap);
        DiameterCounts = new Dictionary<double, int>(diaMap);
    }
}
