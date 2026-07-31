using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using GalvoStage.Core.Geometry;

namespace GalvoStage.App.Rendering;

/// <summary>
/// 场景几何缓存：每条轮廓的包围盒（扁平 float 数组）与全局包围盒/总长/顶点数。
/// 导入时一次并行构建，供视口裁剪、点云 LOD 与 FitView 使用，
/// 避免每帧或每次自适应视图时全量扫描上亿顶点。
/// </summary>
public sealed class SceneGeometryCache
{
    // 每条轮廓的包围盒（与轮廓列表同序）
    public readonly float[] MinX;
    public readonly float[] MinY;
    public readonly float[] MaxX;
    public readonly float[] MaxY;

    public int Count => MinX.Length;

    public double WorldMinX { get; private set; } = double.MaxValue;
    public double WorldMinY { get; private set; } = double.MaxValue;
    public double WorldMaxX { get; private set; } = double.MinValue;
    public double WorldMaxY { get; private set; } = double.MinValue;
    public bool HasBounds => WorldMinX <= WorldMaxX;

    public long VertexCount { get; private set; }
    public double TotalLength { get; private set; }

    /// <summary>各图层的轮廓数（图层名 → 数量），随统计遍历一次性得出</summary>
    public IReadOnlyDictionary<string, int> LayerCounts => _layerCounts;
    private readonly Dictionary<string, int> _layerCounts = new();

    private SceneGeometryCache(int count)
    {
        MinX = new float[count];
        MinY = new float[count];
        MaxX = new float[count];
        MaxY = new float[count];
    }

    /// <summary>并行遍历一次轮廓集合，同时得到逐条包围盒与全局统计。</summary>
    public static SceneGeometryCache Build(IReadOnlyList<PathPolyline> polylines)
    {
        var c = new SceneGeometryCache(polylines.Count);
        object gate = new();
        Parallel.For(0, polylines.Count,
            () => (minX: double.MaxValue, minY: double.MaxValue,
                   maxX: double.MinValue, maxY: double.MinValue, len: 0.0, verts: 0L,
                   layers: new Dictionary<string, int>()),
            (i, _, acc) =>
            {
                var pl = polylines[i];
                var pts = pl.Points;
                double mnx = double.MaxValue, mny = double.MaxValue;
                double mxx = double.MinValue, mxy = double.MinValue;
                double len = 0;
                for (int k = 0; k < pts.Count; k++)
                {
                    Vec2 p = pts[k];
                    if (p.X < mnx) mnx = p.X;
                    if (p.X > mxx) mxx = p.X;
                    if (p.Y < mny) mny = p.Y;
                    if (p.Y > mxy) mxy = p.Y;
                    if (k > 0) len += p.DistanceTo(pts[k - 1]);
                }
                if (pl.Closed && pts.Count > 1) len += pts[0].DistanceTo(pts[^1]);

                c.MinX[i] = (float)mnx;
                c.MinY[i] = (float)mny;
                c.MaxX[i] = (float)mxx;
                c.MaxY[i] = (float)mxy;

                if (mnx < acc.minX) acc.minX = mnx;
                if (mny < acc.minY) acc.minY = mny;
                if (mxx > acc.maxX) acc.maxX = mxx;
                if (mxy > acc.maxY) acc.maxY = mxy;
                acc.len += len;
                acc.verts += pts.Count;
                acc.layers.TryGetValue(pl.Layer, out int lc);
                acc.layers[pl.Layer] = lc + 1;
                return acc;
            },
            acc =>
            {
                lock (gate)
                {
                    if (acc.minX < c.WorldMinX) c.WorldMinX = acc.minX;
                    if (acc.minY < c.WorldMinY) c.WorldMinY = acc.minY;
                    if (acc.maxX > c.WorldMaxX) c.WorldMaxX = acc.maxX;
                    if (acc.maxY > c.WorldMaxY) c.WorldMaxY = acc.maxY;
                    c.TotalLength += acc.len;
                    c.VertexCount += acc.verts;
                    foreach (var kv in acc.layers)
                    {
                        c._layerCounts.TryGetValue(kv.Key, out int lc);
                        c._layerCounts[kv.Key] = lc + kv.Value;
                    }
                }
            });
        return c;
    }
}
