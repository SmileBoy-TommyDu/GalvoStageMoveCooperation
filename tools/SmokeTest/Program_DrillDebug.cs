using System;
using GalvoStage.Core.Dxf;

namespace DrillDebug
{
    class Program
    {
        static void Main(string[] args)
        {
            string dxfPath = @"src\GalvoStage.App\Samples\test-panel-600w.dxf";
            
            Console.WriteLine("=== Debug GroupReader ===");
            
            // 使用原有的 DxfParser.ParseFile 看看能不能正常读取
            var polylines = DxfParser.ParseFile(dxfPath);
            Console.WriteLine($"DxfParser.ParseFile: {polylines.Count} 条折线");
            
            if (polylines.Count > 0)
            {
                Console.WriteLine($"\n前 3 条折线的点数:");
                for (int i = 0; i < Math.Min(3, polylines.Count); i++)
                    Console.WriteLine($"   #{i}: {polylines[i].Points.Count} 个点");
            }
        }
    }
}
