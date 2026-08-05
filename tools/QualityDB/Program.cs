using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using GalvoStage.Core.Drilling;
using GalvoStage.Core.Geometry.Drilling;

namespace QualityDB;

/// <summary>
/// 加工质量记录条目
/// </summary>
public sealed class QualityRecord
{
    public int Id { get; init; }
    public DateTime Timestamp { get; init; }
    
    // 材料信息
    public MaterialType Material { get; init; }
    public double Thickness { get; init; }
    
    // 孔信息
    public double HoleDiameter { get; init; }
    public string HoleType { get; init; } = "";
    
    // 工艺参数
    public double Power { get; init; }
    public int OffsetRings { get; init; }
    public double FeedRate { get; init; }
    public double HoldTime { get; init; }
    public double CoolDownInterval { get; init; }
    public double DutyCycle { get; init; }
    
    // 质量评估（实测后填写）
    public double QualityScore { get; set; }  // 0-100 分
    public bool HasBurning { get; set; }      // 是否过烧
    public bool HasIncompleteCut { get; set; } // 是否切不透
    public bool HasDeformation { get; set; }   // 是否变形
    public double ActualDiameter { get; set; } // 实际孔径
    public double DiameterDeviation { get; set; } // 孔径偏差
    public string Notes { get; set; } = "";    // 备注
    
    public override string ToString()
    {
        return $"#{Id} {Timestamp:yyyy-MM-dd HH:mm} | {Material} {Thickness}mm | D={HoleDiameter}mm | " +
               $"P={Power}W R={OffsetRings} F={FeedRate}mm/s | 质量={QualityScore:F1}";
    }
}

/// <summary>
/// 加工质量数据库管理器
/// </summary>
public sealed class QualityDatabase
{
    private readonly string _dbPath;
    private readonly List<QualityRecord> _records = new();
    private int _nextId = 1;
    
    public QualityDatabase(string dbPath = "quality_database.json")
    {
        _dbPath = dbPath;
        Load();
    }
    
    /// <summary>
    /// 添加新记录
    /// </summary>
    public QualityRecord AddRecord(
        MaterialType material, double thickness, double holeDiameter,
        TrepanParams @params, string holeType = "")
    {
        var record = new QualityRecord
        {
            Id = _nextId++,
            Timestamp = DateTime.Now,
            Material = material,
            Thickness = thickness,
            HoleDiameter = holeDiameter,
            HoleType = holeType,
            Power = @params.Power,
            OffsetRings = @params.OffsetRings,
            FeedRate = @params.FeedRate,
            HoldTime = @params.HoldTime,
            CoolDownInterval = @params.CoolDownInterval,
            DutyCycle = @params.DutyCycle,
            QualityScore = 0,  // 待实测
            HasBurning = false,
            HasIncompleteCut = false,
            HasDeformation = false,
            ActualDiameter = holeDiameter,
            DiameterDeviation = 0,
            Notes = ""
        };
        
        _records.Add(record);
        Save();
        
        return record;
    }
    
    /// <summary>
    /// 更新记录（实测后填写质量数据）
    /// </summary>
    public void UpdateRecord(int id, double qualityScore, bool hasBurning, 
        bool hasIncompleteCut, bool hasDeformation, double actualDiameter, string notes = "")
    {
        var record = _records.FirstOrDefault(r => r.Id == id);
        if (record != null)
        {
            record.QualityScore = qualityScore;
            record.HasBurning = hasBurning;
            record.HasIncompleteCut = hasIncompleteCut;
            record.HasDeformation = hasDeformation;
            record.ActualDiameter = actualDiameter;
            record.DiameterDeviation = actualDiameter - record.HoleDiameter;
            record.Notes = notes;
            Save();
        }
    }
    
    /// <summary>
    /// 查询指定材料和孔径的最佳参数
    /// </summary>
    public TrepanParams? GetBestParams(MaterialType material, double thickness, double holeDiameter)
    {
        var matchingRecords = _records
            .Where(r => r.Material == material && 
                       Math.Abs(r.Thickness - thickness) < 0.1 &&
                       Math.Abs(r.HoleDiameter - holeDiameter) < 0.1 &&
                       r.QualityScore > 0)
            .OrderByDescending(r => r.QualityScore)
            .ToList();
        
        if (matchingRecords.Count == 0) return null;
        
        var best = matchingRecords[0];
        return TrepanParams.Custom(
            best.Power, best.OffsetRings, best.FeedRate, 
            best.HoldTime, best.CoolDownInterval, best.DutyCycle);
    }
    
    /// <summary>
    /// 获取统计信息
    /// </summary>
    public (int TotalCount, double AvgQuality, int BurningCount, int IncompleteCount) GetStatistics()
    {
        int total = _records.Count;
        if (total == 0) return (0, 0, 0, 0);
        
        double avgQuality = _records.Average(r => r.QualityScore);
        int burning = _records.Count(r => r.HasBurning);
        int incomplete = _records.Count(r => r.HasIncompleteCut);
        
        return (total, avgQuality, burning, incomplete);
    }
    
    /// <summary>
    /// 导出报告
    /// </summary>
    public void ExportReport(string filePath)
    {
        using var writer = new StreamWriter(filePath);
        
        writer.WriteLine("=== 加工质量数据库报告 ===");
        writer.WriteLine($"生成时间：{DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        writer.WriteLine($"总记录数：{_records.Count}\n");
        
        var stats = GetStatistics();
        writer.WriteLine("统计信息：");
        writer.WriteLine($"  平均质量得分：{stats.AvgQuality:F1}/100");
        writer.WriteLine($"  过烧孔数：{stats.BurningCount}");
        writer.WriteLine($"  切不透孔数：{stats.IncompleteCount}\n");
        
        writer.WriteLine("详细记录：");
        foreach (var r in _records.OrderByDescending(r => r.QualityScore))
        {
            writer.WriteLine(r.ToString());
            if (!string.IsNullOrEmpty(r.Notes))
                writer.WriteLine($"  备注：{r.Notes}");
        }
    }
    
    private void Save()
    {
        var options = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase
        };
        
        string json = JsonSerializer.Serialize(_records, options);
        File.WriteAllText(_dbPath, json);
    }
    
    private void Load()
    {
        if (!File.Exists(_dbPath)) return;
        
        try
        {
            string json = File.ReadAllText(_dbPath);
            var options = new JsonSerializerOptions
            {
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase
            };
            var loaded = JsonSerializer.Deserialize<List<QualityRecord>>(json, options);
            if (loaded != null)
            {
                _records.Clear();
                _records.AddRange(loaded);
                _nextId = _records.Count > 0 ? _records.Max(r => r.Id) + 1 : 1;
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"加载数据库失败：{ex.Message}");
        }
    }
}

/// <summary>
/// 演示程序
/// </summary>
class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("=== 加工质量数据库管理系统 ===\n");
        
        var db = new QualityDatabase("quality_db.json");
        
        // 添加一些示例记录
        Console.WriteLine("添加示例记录...");
        
        // FR-4 1.6mm 标准板，微孔
        var record1 = db.AddRecord(
            MaterialType.FR4, 1.6, 0.5,
            TrepanParams.SmallHole, "微孔");
        Console.WriteLine($"  添加：{record1}");
        
        // FR-4 1.6mm 标准板，中孔
        var record2 = db.AddRecord(
            MaterialType.FR4, 1.6, 3.0,
            TrepanParams.LargeHole, "中孔");
        Console.WriteLine($"  添加：{record2}");
        
        // 铝基板 2.0mm，大孔
        var record3 = db.AddRecord(
            MaterialType.Aluminum, 2.0, 6.0,
            TrepanParams.Custom(13000, 5, 75, 55, 170, 0.65), "大孔");
        Console.WriteLine($"  添加：{record3}");
        
        // 模拟实测数据更新
        Console.WriteLine("\n模拟实测数据更新...");
        db.UpdateRecord(record1.Id, 
            qualityScore: 95.5,
            hasBurning: false,
            hasIncompleteCut: false,
            hasDeformation: false,
            actualDiameter: 0.52,
            notes: "质量良好，无过烧");
        Console.WriteLine($"  更新 #{record1.Id}: 质量={95.5:F1}");
        
        db.UpdateRecord(record2.Id,
            qualityScore: 88.0,
            hasBurning: false,
            hasIncompleteCut: false,
            hasDeformation: false,
            actualDiameter: 3.05,
            notes: "切透，边缘光滑");
        Console.WriteLine($"  更新 #{record2.Id}: 质量={88.0:F1}");
        
        db.UpdateRecord(record3.Id,
            qualityScore: 92.3,
            hasBurning: false,
            hasIncompleteCut: false,
            hasDeformation: false,
            actualDiameter: 6.08,
            notes: "铝基板加工正常");
        Console.WriteLine($"  更新 #{record3.Id}: 质量={92.3:F1}");
        
        // 查询最佳参数
        Console.WriteLine("\n查询最佳参数...");
        var bestParams = db.GetBestParams(MaterialType.FR4, 1.6, 0.5);
        if (bestParams != null)
        {
            Console.WriteLine($"  FR-4 1.6mm 微孔最佳参数：{bestParams}");
        }
        
        // 统计信息
        var stats = db.GetStatistics();
        Console.WriteLine($"\n统计信息：");
        Console.WriteLine($"  总记录数：{stats.TotalCount}");
        Console.WriteLine($"  平均质量：{stats.AvgQuality:F1}/100");
        Console.WriteLine($"  过烧孔数：{stats.BurningCount}");
        Console.WriteLine($"  切不透孔数：{stats.IncompleteCount}");
        
        // 导出报告
        db.ExportReport("quality_report_detailed.txt");
        Console.WriteLine("\n详细报告已导出到：quality_report_detailed.txt");
        
        Console.WriteLine("\n=== 数据库管理完成 ===");
    }
}
