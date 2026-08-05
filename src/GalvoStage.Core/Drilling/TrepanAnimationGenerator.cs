using System;
using System.Collections.Generic;
using GalvoStage.Core.Geometry;
using GalvoStage.Core.Geometry.Drilling;

namespace GalvoStage.Core.Drilling;

/// <summary>
/// 环切轨迹动画生成器
/// 为每个孔生成环切轨迹的采样点序列，用于仿真动画
/// </summary>
public sealed class TrepanAnimationGenerator
{
    /// <summary>
    /// 单个孔的环切轨迹点
    /// </summary>
    public sealed class TrepanPoint
    {
        public Vec2 Position { get; init; }
        public double Power { get; init; }
        public int RingIndex { get; init; }      // 当前圈数（0-based）
        public double Progress { get; init; }     // 进度 (0.0-1.0)
        public bool IsLaserOn { get; init; }
    }
    
    /// <summary>
    /// 为单个孔生成环切轨迹动画点
    /// </summary>
    /// <param name="centerX">孔中心 X</param>
    /// <param name="centerY">孔中心 Y</param>
    /// <param name="diameter">孔径</param>
    /// <param name="params">工艺参数</param>
    /// <param name="samplesPerRing">每圈采样点数</param>
    /// <returns>轨迹点序列</returns>
    public static IEnumerable<TrepanPoint> GenerateTrepanAnimation(
        double centerX, double centerY, double diameter, TrepanParams @params,
        int samplesPerRing = 36)
    {
        if (@params == null || @params.OffsetRings <= 0) yield break;
        
        double radius = diameter > 0 ? diameter / 2.0 : 0.5;
        
        // 逐圈生成轨迹
        for (int ring = 0; ring < @params.OffsetRings; ring++)
        {
            double ringRadius = radius * (ring + 1) / @params.OffsetRings;
            
            // 生成圆周上的采样点
            for (int i = 0; i <= samplesPerRing; i++)
            {
                double angle = 2 * Math.PI * i / samplesPerRing;
                double x = centerX + ringRadius * Math.Cos(angle);
                double y = centerY + ringRadius * Math.Sin(angle);
                
                yield return new TrepanPoint
                {
                    Position = new Vec2(x, y),
                    Power = @params.Power,
                    RingIndex = ring,
                    Progress = (double)i / samplesPerRing,
                    IsLaserOn = true
                };
            }
            
            // 冷却间隔（模拟暂停）
            if (@params.CoolDownInterval > 0 && ring < @params.OffsetRings - 1)
            {
                // 生成一个激光关闭的点表示冷却
                yield return new TrepanPoint
                {
                    Position = new Vec2(centerX, centerY),
                    Power = 0,
                    RingIndex = ring,
                    Progress = 1.0,
                    IsLaserOn = false
                };
            }
        }
        
        // 持留时间（最后停留在中心）
        if (@params.HoldTime > 0)
        {
            yield return new TrepanPoint
            {
                Position = new Vec2(centerX, centerY),
                Power = @params.Power * 0.5,  // 持留时功率降低
                RingIndex = @params.OffsetRings - 1,
                Progress = 1.0,
                IsLaserOn = true
            };
        }
    }
    
    /// <summary>
    /// 为整个钻孔模式生成动画帧序列
    /// </summary>
    public static List<List<TrepanPoint>> GenerateAnimationFrames(
        DrillingPattern pattern, int samplesPerRing = 36)
    {
        var frames = new List<List<TrepanPoint>>();
        
        foreach (var hole in pattern.Holes)
        {
            if (hole.ProcessParams == null) continue;
            
            var points = new List<TrepanPoint>();
            points.AddRange(GenerateTrepanAnimation(
                hole.X, hole.Y, hole.Diameter, hole.ProcessParams, samplesPerRing));
            
            frames.Add(points);
        }
        
        return frames;
    }
    
    /// <summary>
    /// 计算动画总时长（毫秒）
    /// </summary>
    public static double CalculateTotalDuration(DrillingPattern pattern)
    {
        double totalMs = 0;
        
        foreach (var hole in pattern.Holes)
        {
            if (hole.ProcessParams == null) continue;
            
            var pp = hole.ProcessParams;
            
            // 每圈时间 ≈ 周长 / 进给速度 * 1000 (转 ms)
            double circumference = Math.PI * hole.Diameter;
            double timePerRing = circumference / pp.FeedRate * 1000;
            totalMs += timePerRing * pp.OffsetRings;
            
            // 冷却时间
            totalMs += pp.CoolDownInterval * (pp.OffsetRings - 1);
            
            // 持留时间
            totalMs += pp.HoldTime;
        }
        
        return totalMs;
    }
}
