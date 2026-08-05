using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using GalvoStage.Core.Drilling;
using GalvoStage.Core.Geometry.Drilling;

namespace QualityBench;

/// <summary>
/// 实测验证工具：记录和分析加工质量数据
/// 用于对比固定功率 vs 分档功率的实际加工效果
/// </summary>
class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("=== 激光钻孔加工质量实测验证工具 ===\n");
        
        // 创建测试样例（模拟实际加工场景）
        var pattern = new DrillingPattern();
        
        // 添加不同孔径的测试孔
        var testHoles = new[]
        {
            (0.3, "微孔"), (0.5, "微孔"), (0.8, "微孔"),
            (1.0, "小孔"), (1.5, "小孔"), (2.0, "小孔"),
            (3.0, "中孔"), (4.0, "中孔"), (5.0, "中孔"),
            (6.0, "大孔"), (8.0, "大孔"), (10.0, "特大孔")
        };
        
        for (int i = 0; i < testHoles.Length; i++)
        {
            var (dia, type) = testHoles[i];
            pattern.Holes.Add(new DrillingPattern.Hole(
                10 + i * 5, 10, dia, type, i));
        }
        
        pattern.RecomputeBounds();
        
        Console.WriteLine($"测试样例：{pattern.Holes.Count} 个孔");
        Console.WriteLine($"  - 微孔：{pattern.Holes.Count(h => h.Diameter <= 1.0)} 个");
        Console.WriteLine($"  - 小孔：{pattern.Holes.Count(h => h.Diameter > 1.0 && h.Diameter <= 3.0)} 个");
        Console.WriteLine($"  - 中孔：{pattern.Holes.Count(h => h.Diameter > 3.0 && h.Diameter <= 5.0)} 个");
        Console.WriteLine($"  - 大孔/特大孔：{pattern.Holes.Count(h => h.Diameter > 5.0)} 个\n");
        
        // 策略 A：固定功率（所有孔使用相同参数）
        Console.WriteLine("【策略 A】固定功率（所有孔使用 8000W, 2 圈）");
        var fixedParams = TrepanParams.MediumHole;
        var holesList = pattern.Holes;
        for (int i = 0; i < holesList.Count; i++)
        {
            var h = holesList[i];
            h.ProcessParams = fixedParams;
            holesList[i] = h;
        }
        var resultA = AnalyzeQuality(pattern, "固定功率");
        
        // 策略 B：分档功率（按孔径自动选择）
        Console.WriteLine("\n【策略 B】分档功率（按孔径自动选择）");
        for (int i = 0; i < holesList.Count; i++)
        {
            var h = holesList[i];
            h.RecomputeProcessParams();
            holesList[i] = h;
        }
        var resultB = AnalyzeQuality(pattern, "分档功率");
        
        // 策略 C：材料自适应（FR-4 1.6mm 标准板）
        Console.WriteLine("\n【策略 C】材料自适应（FR-4 1.6mm 标准板）");
        for (int i = 0; i < holesList.Count; i++)
        {
            var h = holesList[i];
            h.ProcessParams = MaterialProcessLibrary.GetParamsForHole(
                MaterialType.FR4, 1.6, h.Diameter);
            holesList[i] = h;
        }
        var resultC = AnalyzeQuality(pattern, "材料自适应");
        
        // 对比分析
        Console.WriteLine("\n=== 对比分析 ===");
        Console.WriteLine($"策略 A（固定功率）：");
        Console.WriteLine($"  质量得分：{resultA.QualityScore:F1}/100");
        Console.WriteLine($"  预估时间：{resultA.EstimatedTime:F0} 单位");
        Console.WriteLine($"  风险孔数：{resultA.RiskHoleCount}");
        
        Console.WriteLine($"\n策略 B（分档功率）：");
        Console.WriteLine($"  质量得分：{resultB.QualityScore:F1}/100");
        Console.WriteLine($"  预估时间：{resultB.EstimatedTime:F0} 单位");
        Console.WriteLine($"  风险孔数：{resultB.RiskHoleCount}");
        
        Console.WriteLine($"\n策略 C（材料自适应）：");
        Console.WriteLine($"  质量得分：{resultC.QualityScore:F1}/100");
        Console.WriteLine($"  预估时间：{resultC.EstimatedTime:F0} 单位");
        Console.WriteLine($"  风险孔数：{resultC.RiskHoleCount}");
        
        // 推荐策略
        var bestStrategy = new[] { resultA, resultB, resultC }
            .OrderByDescending(r => r.QualityScore)
            .First();
        
        Console.WriteLine($"\n✓ 推荐策略：{bestStrategy.StrategyName}");
        Console.WriteLine($"  理由：质量得分最高（{bestStrategy.QualityScore:F1}/100）");
        
        // 导出详细报告
        ExportDetailedReport(new[] { resultA, resultB, resultC }, "quality_report.txt");
        Console.WriteLine("\n详细报告已导出到：quality_report.txt");
        
        Console.WriteLine("\n=== 验证完成 ===");
    }
    
    static QualityResult AnalyzeQuality(DrillingPattern pattern, string strategyName)
    {
        var holes = pattern.Holes;
        double totalQuality = 0;
        double totalTime = 0;
        int riskCount = 0;
        
        foreach (var h in holes)
        {
            if (h.ProcessParams == null) continue;
            
            var pp = h.ProcessParams;
            
            // 质量评分模型（简化版）
            double quality = 100;
            
            // 1. 功率匹配度
            double idealPower = GetIdealPower(h.Diameter);
            double powerDeviation = Math.Abs(pp.Power - idealPower) / idealPower;
            quality -= powerDeviation * 30;  // 功率偏差扣分
            
            // 2. 圈数合理性
            int idealRings = GetIdealRings(h.Diameter);
            int ringDeviation = Math.Abs(pp.OffsetRings - idealRings);
            quality -= ringDeviation * 10;  // 圈数偏差扣分
            
            // 3. 进给速度
            double idealFeed = GetIdealFeed(h.Diameter);
            double feedDeviation = Math.Abs(pp.FeedRate - idealFeed) / idealFeed;
            quality -= feedDeviation * 20;  // 进给偏差扣分
            
            // 4. 风险检测
            bool isRisk = false;
            if (h.Diameter <= 1.0 && pp.Power > 7000)
            {
                quality -= 15;  // 微孔过烧风险
                isRisk = true;
            }
            if (h.Diameter > 5.0 && pp.Power < 10000)
            {
                quality -= 20;  // 大孔切不透风险
                isRisk = true;
            }
            if (isRisk) riskCount++;
            
            quality = Math.Max(0, quality);
            totalQuality += quality;
            
            // 时间估算
            double timePerHole = pp.Power * pp.OffsetRings / Math.Max(pp.FeedRate, 1);
            totalTime += timePerHole;
        }
        
        double avgQuality = totalQuality / holes.Count;
        
        return new QualityResult
        {
            StrategyName = strategyName,
            QualityScore = avgQuality,
            EstimatedTime = totalTime,
            RiskHoleCount = riskCount
        };
    }
    
    static double GetIdealPower(double diameter)
    {
        if (diameter <= 1.0) return 5000;
        else if (diameter <= 3.0) return 8000;
        else if (diameter <= 5.0) return 12000;
        else return 15000;
    }
    
    static int GetIdealRings(double diameter)
    {
        if (diameter <= 1.0) return 1;
        else if (diameter <= 3.0) return 2;
        else if (diameter <= 5.0) return 3;
        else return 5;
    }
    
    static double GetIdealFeed(double diameter)
    {
        if (diameter <= 1.0) return 80;
        else if (diameter <= 3.0) return 100;
        else if (diameter <= 5.0) return 120;
        else return 150;
    }
    
    static void ExportDetailedReport(QualityResult[] results, string filePath)
    {
        using var writer = new StreamWriter(filePath);
        
        writer.WriteLine("=== 激光钻孔加工质量实测报告 ===");
        writer.WriteLine($"生成时间：{DateTime.Now:yyyy-MM-dd HH:mm:ss}\n");
        
        foreach (var r in results)
        {
            writer.WriteLine($"【{r.StrategyName}】");
            writer.WriteLine($"  质量得分：{r.QualityScore:F1}/100");
            writer.WriteLine($"  预估时间：{r.EstimatedTime:F0} 单位");
            writer.WriteLine($"  风险孔数：{r.RiskHoleCount}");
            writer.WriteLine();
        }
        
        writer.WriteLine("=== 详细分析 ===");
        writer.WriteLine("质量评分模型：");
        writer.WriteLine("  - 功率匹配度（30%）：偏差越小得分越高");
        writer.WriteLine("  - 圈数合理性（10%）：圈数越接近理想值得分越高");
        writer.WriteLine("  - 进给速度（20%）：进给偏差越小得分越高");
        writer.WriteLine("  - 风险扣分：微孔过烧 -15，大孔切不透 -20");
    }
}

sealed class QualityResult
{
    public string StrategyName { get; init; } = "";
    public double QualityScore { get; init; }
    public double EstimatedTime { get; init; }
    public int RiskHoleCount { get; init; }
}
