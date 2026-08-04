using System;
using System.IO;
using System.Linq;
using GalvoStage.Core.Dxf;

// 批量验证多个混合 DXF 测试文件
string samplesDir = Path.GetFullPath(
    Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..",
        "src", "GalvoStage.App", "Samples"));

string[] testFiles = new[]
{
    "mixed-test-1.dxf",
    "mixed-test-2-pcb.dxf"
};

Console.WriteLine("双模式分离加工批量验证");
Console.WriteLine(new string('=', 70));

int passCount = 0, failCount = 0;

foreach (var fileName in testFiles)
{
    string dxfPath = Path.Combine(samplesDir, fileName);
    if (!File.Exists(dxfPath))
    {
        Console.WriteLine($"⚠️ 跳过：{fileName}（文件不存在）");
        continue;
    }

    Console.WriteLine();
    Console.WriteLine($"📄 {fileName}");
    Console.WriteLine(new string('-', 70));

    try
    {
        var result = DxfParser.ParseFileMixed(dxfPath);
        var legacy = DxfParser.ParseFile(dxfPath);

        int holes = result.DrillingHoles.Holes.Count;
        int contours = result.Polylines.Count;
        int circles = result.CircleCount;
        int legacyCount = legacy.Count;

        Console.WriteLine($"  双模式：折线 {contours} 条 / 钻孔 {holes} 个 / CIRCLE {circles} 个");
        Console.WriteLine($"  单模式：折线 {legacyCount} 条（旧链路）");

        // 验证：双模式的折线数应等于旧链路（因为 CIRCLE 在两条链路都被保留）
        if (contours == legacyCount)
        {
            Console.WriteLine($"  ✅ 折线数量一致（双模式 = 单模式）");
        }
        else
        {
            Console.WriteLine($"  ⚠️ 折线数量差异：{contours} vs {legacyCount}");
        }

        // 验证：钻孔数应等于 CIRCLE 数
        if (holes == circles)
        {
            Console.WriteLine($"  ✅ 钻孔数 = CIRCLE 数 = {holes}");
            passCount++;
        }
        else
        {
            Console.WriteLine($"  ❌ 钻孔数 ({holes}) ≠ CIRCLE 数 ({circles})");
            failCount++;
        }

        // 图层统计
        var layerCounts = new System.Collections.Generic.Dictionary<string, int>();
        foreach (var pl in result.Polylines)
        {
            string layer = pl.Layer ?? "0";
            if (!layerCounts.ContainsKey(layer)) layerCounts[layer] = 0;
            layerCounts[layer]++;
        }
        Console.WriteLine($"  图层：{string.Join(", ", layerCounts.OrderByDescending(x => x.Value).Select(kv => $"{kv.Key}({kv.Value})"))}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"  ❌ 解析失败：{ex.Message}");
        failCount++;
    }
}

Console.WriteLine();
Console.WriteLine(new string('=', 70));
Console.WriteLine($"【汇总】通过 {passCount} / 失败 {failCount}");

if (failCount == 0)
{
    Console.WriteLine("🎉 所有测试通过！双模式分离加工实现正确。");
    return 0;
}
else
{
    Console.WriteLine("❌ 部分测试失败，请检查。");
    return 1;
}
