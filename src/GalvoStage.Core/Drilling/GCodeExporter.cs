using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace GalvoStage.Core.Drilling;

/// <summary>
/// 钻孔轨迹 → 标准 G 代码导出器
/// 按孔径分组换刀（T1/T2…），孔位使用 G81 固定钻孔循环，流式写出支持百万级孔
/// </summary>
public static class GCodeExporter
{
    /// <summary>导出参数</summary>
    public sealed class Options
    {
        public double SafeZ = 5.0;          // 安全高度 (mm)
        public double RetractZ = 0.5;       // 循环回退平面 R (mm)
        public double DrillDepth = -1.6;    // 钻孔深度 Z (mm，负值向下)
        public double FeedRate = 300;       // 进给速度 (mm/min)
        public int SpindleRpm = 30000;      // 主轴转速
        public string ProgramNumber = "O0001";
    }

    /// <summary>
    /// 将钻孔轨迹写为 G 代码文件。
    /// 孔按孔径分组输出（每组一次换刀 + M0 暂停确认），组内保持规划顺序。
    /// 返回写出的总孔数。
    /// </summary>
    public static int Export(DrillPlanner.DrillingTrajectory trajectory, string path, Options? options = null)
    {
        var opt = options ?? new Options();
        var ci = CultureInfo.InvariantCulture;
        var moves = trajectory.Moves;

        // 按孔径分组（保持组内规划顺序），未知孔径(0)归为一组放最后
        var groups = moves
            .GroupBy(m => Math.Round(m.Diameter, 3))
            .OrderBy(g => g.Key == 0 ? double.MaxValue : g.Key)
            .ToList();

        int total = 0;
        using var w = new StreamWriter(path, false, System.Text.Encoding.ASCII, 1 << 20);

        w.WriteLine("%");
        w.WriteLine($"{opt.ProgramNumber} (GalvoStage PCB Drill - {moves.Count} holes, {groups.Count} tools)");
        w.WriteLine("G21 G90 G94 (mm / absolute / feed per min)");
        w.WriteLine("G17");

        int toolNo = 0;
        foreach (var g in groups)
        {
            toolNo++;
            var holes = g.ToList();
            string diaLabel = g.Key > 0 ? $"D{g.Key.ToString("F3", ci)}mm" : "D-unknown";
            w.WriteLine();
            w.WriteLine($"(TOOL {toolNo}: {diaLabel}, {holes.Count} holes)");
            w.WriteLine($"T{toolNo} M6");
            w.WriteLine("M0 (confirm tool then cycle start)");
            w.WriteLine($"M3 S{opt.SpindleRpm}");
            w.WriteLine($"G0 Z{opt.SafeZ.ToString("F3", ci)}");

            bool first = true;
            foreach (var m in holes)
            {
                string x = m.Position.X.ToString("F4", ci);
                string y = m.Position.Y.ToString("F4", ci);
                if (first)
                {
                    // 首孔：快移定位后启动 G81 固定循环
                    w.WriteLine($"G0 X{x} Y{y}");
                    w.WriteLine($"G81 X{x} Y{y} Z{opt.DrillDepth.ToString("F3", ci)} " +
                                $"R{opt.RetractZ.ToString("F3", ci)} F{opt.FeedRate.ToString("F0", ci)}");
                    first = false;
                }
                else
                {
                    // 后续孔：循环模态下仅给坐标
                    w.WriteLine($"X{x} Y{y}");
                }
                total++;
            }
            w.WriteLine("G80");
            w.WriteLine("M5");
        }

        w.WriteLine();
        w.WriteLine($"G0 Z{opt.SafeZ.ToString("F3", ci)}");
        w.WriteLine("M30");
        w.WriteLine("%");
        return total;
    }
}
