using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using GalvoStage.Core.Geometry;

namespace GalvoStage.Core.Dxf;

/// <summary>PCB 钻孔 DXF 解析器 - 简化版本（提取 CIRCLE 圆心 + 孔径）</summary>
public static class DrillingDxfParser
{
    /// <summary>
    /// 从文件解析钻孔点集。
    /// layerFilter 非空时仅保留指定图层的孔（忽略大小写），用于超大文件按图层分批导入、降低内存占用。
    /// </summary>
    public static Geometry.Drilling.DrillingPattern ParseFile(string path, string? layerFilter = null)
    {
        byte[] data = File.ReadAllBytes(path);
        string content = System.Text.Encoding.UTF8.GetString(data);
        
        var pattern = new Geometry.Drilling.DrillingPattern();
        
        // 简单的循环扫描提取 CIRCLE 圆心
        string[] lines = content.Split('\n');
        List<string> layerStack = new();
        
        for (int i = 0; i < lines.Length; i++)
        {
            string line = lines[i].Trim().ToLowerInvariant();
            
            if (line == "circle")
            {
                double x = 0, y = 0, radius = 0;
                string layer = layerStack.Count > 0 ? layerStack[layerStack.Count - 1] : "0";
                
                // 读取后续组码：兼容“码 值”同行与标准分行两种格式，遇组码 0（下一实体）停止
                int j = i + 1;
                int limit = Math.Min(i + 24, lines.Length - 1);
                for (; j <= limit; j++)
                {
                    string sub = lines[j].Trim();
                    string[] parts = sub.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length == 0) continue;
                    if (!int.TryParse(parts[0], out int code)) continue;
                    if (code == 0) break;   // 下一实体开始
                    
                    string? value = null;
                    if (parts.Length >= 2)
                    {
                        value = parts[1];               // 同行格式：code value
                    }
                    else if (j + 1 <= lines.Length - 1)
                    {
                        value = lines[j + 1].Trim();    // 标准分行格式：值在下一行
                        j++;
                    }
                    if (value == null) continue;
                    
                    try
                    {
                        if (code == 10) x = double.Parse(value, CultureInfo.InvariantCulture);
                        else if (code == 20) y = double.Parse(value, CultureInfo.InvariantCulture);
                        else if (code == 40) radius = double.Parse(value, CultureInfo.InvariantCulture);
                        else if (code == 8) layer = value;
                    }
                    catch { }
                }
                
                if (layerFilter == null || layer.Equals(layerFilter, StringComparison.OrdinalIgnoreCase))
                    pattern.Holes.Add(new Geometry.Drilling.DrillingPattern.Hole(
                        x, y, radius * 2, layer, pattern.Holes.Count));
                i = j - 1; // 从下一实体起点继续扫描
            }
            else if (line.StartsWith("layer "))
            {
                string[] parts = line.Split(' ');
                if (parts.Length > 1 && !string.IsNullOrWhiteSpace(parts[1]))
                {
                    layerStack.Add(parts[1]);
                }
            }
            else if (line == "endlayer")
            {
                if (layerStack.Count > 0) layerStack.RemoveAt(layerStack.Count - 1);
            }
        }
        
        pattern.RecomputeBounds();
        return pattern;
    }
}
