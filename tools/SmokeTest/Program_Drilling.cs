using System;
using GalvoStage.Core.Dxf;
using Geometry = GalvoStage.Core.Geometry;
using GalvoStage.Core.Drilling;

namespace DrillingTest
{
    class Program
    {
        static void Main(string[] args)
        {
            string dxfPath = args.Length > 0 ? args[0] : @"src\GalvoStage.App\Samples\test-panel-600w.dxf";
            string desc = args.Length > 0 ? "custom" : "test-panel-600w";
            
            Console.WriteLine($"=== PCB 钻孔 DXF 解析器 - {desc} ===\n");
            
            try
            {
                // Step 1: 简单统计
                string content = System.IO.File.ReadAllText(dxfPath);
                int circleCount = 0;
                
                foreach (string line in content.Split('\n'))
                {
                    if (line.Trim().ToLowerInvariant() == "circle")
                        circleCount++;
                }
                
                Console.WriteLine($"📊 DXF 概览：CIRCLE 实体数 = {circleCount:N0}\n");
                
                // Step 2: 解析
                Console.WriteLine("正在解析...");
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var pattern = DrillingDxfParser.ParseFile(dxfPath);
                sw.Stop();
                
                Console.WriteLine($"\n✅ 解析完成!");
                Console.WriteLine($"   ⏱️ 耗时：{sw.ElapsedMilliseconds,10} ms");
                Console.WriteLine($"   🕳️ 总孔数：{pattern.Holes.Count:N0}");
                
                if (pattern.Bounds.HasValue)
                {
                    double width = pattern.Bounds.Value.MaxX - pattern.Bounds.Value.MinX;
                    double height = pattern.Bounds.Value.MaxY - pattern.Bounds.Value.MinY;
                    Console.WriteLine($"   📦 包围盒：({width:F2})×({height:F2}) mm\n");
                }
                
                if (pattern.LayerCounts.Count > 0)
                {
                    Console.WriteLine($"📁 图层分布:");
                    foreach (var kv in pattern.LayerCounts)
                        Console.WriteLine($"      - {kv.Key,15}: {kv.Value,12:N0} 个孔");
                    Console.WriteLine();
                }
                
                if (pattern.DiameterCounts.Count > 0)
                {
                    Console.WriteLine($"🔩 孔径分布:");
                    foreach (var kv in pattern.DiameterCounts.OrderByDescending(kv => kv.Value))
                        Console.WriteLine($"      • Ø{kv.Key:F3} mm: {kv.Value:N0} 孔");

                    Console.WriteLine();
                }
                // Step 3: 示例坐标（仅前 3 个）
                if (pattern.Holes.Count > 0)
                {
                    Console.WriteLine($"🔍 示例孔坐标 (前 3 个):");
                    for (int i = 0; i < Math.Min(3, pattern.Holes.Count); i++)
                    {
                        var h = pattern.Holes[i];
                        Console.WriteLine($"      #{i,2}: ({h.X,10:F4}, {h.Y,10:F4}) Ø{h.Diameter:F3} [图层:{h.Layer}]");
                    }
                    Console.WriteLine();
                }
                
                // Step 4: 路径规划（仅规划前 100 个孔用于演示）
                int maxSamples = 100;
                if (pattern.Holes.Count > maxSamples)
                {
                    Console.WriteLine($"⚠️  为演示性能，仅对前{maxSamples}个孔进行路径规划...\n");
                    
                    var samplePattern = new Geometry.Drilling.DrillingPattern();
                    for (int i = 0; i < maxSamples; i++)
                    {
                        var h = pattern.Holes[i];
                        samplePattern.Holes.Add(new Geometry.Drilling.DrillingPattern.Hole(h.X, h.Y, h.Diameter, h.Layer, i));
                    }
                    samplePattern.RecomputeBounds();
                    
                    Console.WriteLine("⚙️ 正在规划路径...");
                    var traceSw = System.Diagnostics.Stopwatch.StartNew();
                    var trajectory = DrillPlanner.Plan(samplePattern);
                    traceSw.Stop();
                                        
                    Console.WriteLine($"\n✅ 路径规划完成!");
                    Console.WriteLine($"   • 移动指令数：{trajectory.Moves.Count:N0}");
                    Console.WriteLine($"   • 规划耗时：{traceSw.ElapsedMilliseconds,8} ms");
                    Console.WriteLine($"   • 预计加工时间：{(double)trajectory.TotalDurationMs / 1000:F1} s");
                    
                    // Step 5: 示例孔输出
                    Console.WriteLine($"\n📍 前 3 个孔详情:");
                    for (int i = 0; i < Math.Min(3, samplePattern.Holes.Count); i++)
                        Console.WriteLine($"   • Hole[{i}] ({samplePattern.Holes[i].Layer}): X={samplePattern.Holes[i].X:F4}, Y={samplePattern.Holes[i].Y:F4}, Ø={samplePattern.Holes[i].Diameter:F3}");
                    
                    // Step 5: G 代码导出验证
                    string gcodePath = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "drill_smoke.nc");
                    int exported = GCodeExporter.Export(trajectory, gcodePath);
                    var gcLines = System.IO.File.ReadAllLines(gcodePath);
                    Console.WriteLine($"\n✅ G 代码导出完成：{exported:N0} 孔 / {gcLines.Length:N0} 行 → {gcodePath}");
                    Console.WriteLine("   前 12 行预览:");
                    for (int i = 0; i < Math.Min(12, gcLines.Length); i++)
                        Console.WriteLine($"      | {gcLines[i]}");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\n❌ 错误：{ex.Message}");
                Console.WriteLine($"   类型：{ex.GetType().Name}");
                if (ex.InnerException != null)
                    Console.WriteLine($"   内层：{ex.InnerException.GetType().Name}: {ex.InnerException.Message}");
            }
        }
    }
}
