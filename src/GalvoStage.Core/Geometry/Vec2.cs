using System;

namespace GalvoStage.Core.Geometry;

/// <summary>二维矢量（单位:mm）</summary>
public readonly struct Vec2 : IEquatable<Vec2>
{
    public readonly double X;
    public readonly double Y;

    public Vec2(double x, double y) { X = x; Y = y; }

    public static readonly Vec2 Zero = new(0, 0);

    public double Length => Math.Sqrt(X * X + Y * Y);

    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator *(Vec2 a, double s) => new(a.X * s, a.Y * s);
    public static Vec2 operator *(double s, Vec2 a) => new(a.X * s, a.Y * s);
    public static Vec2 operator /(Vec2 a, double s) => new(a.X / s, a.Y / s);

    public double DistanceTo(Vec2 other) => (this - other).Length;

    public bool Equals(Vec2 other) => X == other.X && Y == other.Y;
    public override bool Equals(object? obj) => obj is Vec2 v && Equals(v);
    public override int GetHashCode() => HashCode.Combine(X, Y);
    public override string ToString() => $"({X:F3}, {Y:F3})";
}

/// <summary>连续折线（DXF 实体统一细分后的表示）</summary>
public sealed class PathPolyline
{
    public List<Vec2> Points { get; } = new();
    public bool Closed { get; set; }
    public string Layer { get; set; } = "0";

    public double Length
    {
        get
        {
            double len = 0;
            for (int i = 1; i < Points.Count; i++) len += Points[i].DistanceTo(Points[i - 1]);
            if (Closed && Points.Count > 1) len += Points[0].DistanceTo(Points[^1]);
            return len;
        }
    }
}
