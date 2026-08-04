using System;
using System.Buffers.Text;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using GalvoStage.Core.Geometry;

namespace GalvoStage.Core.Dxf;

/// <summary>
/// 轻量级 ASCII DXF 解析器：读取 ENTITIES 段中的常见二维实体，
/// 统一细分为折线（PathPolyline），供后续路径规划使用。
/// 支持：LINE / CIRCLE / ARC / LWPOLYLINE(含凸度) / POLYLINE+VERTEX / ELLIPSE / SPLINE
///
/// 采用单遍字节流式解析：直接在字节缓冲区上解析组码与数值，不为数值分配字符串、
/// 不一次性把全部组码读入内存、也不逐实体分配字典，可在数秒内处理数百 MB 的大文件。
/// 实体所属图层（组码 8）经驻留缓存解码到 <see cref="PathPolyline.Layer"/>，
/// 相同图层名共享同一字符串实例，不随实体数量产生额外分配。
/// </summary>
public static class DxfParser
{
    /// <summary>圆弧细分弦高误差 (mm)</summary>
    public const double ChordTolerance = 0.01;

    public static List<PathPolyline> ParseFile(string path)
    {
        // 整文件读入内存，便于按字节区间随机切分、并行解析
        byte[] data = File.ReadAllBytes(path);
        return ParseBytes(data);
    }

    public static List<PathPolyline> Parse(Stream stream)
    {
        byte[] data;
        if (stream is MemoryStream ms0)
        {
            data = ms0.ToArray();
        }
        else
        {
            using var ms = new MemoryStream();
            stream.CopyTo(ms);
            data = ms.ToArray();
        }
        return ParseBytes(data);
    }

    /// <summary>
    /// 在内存字节缓冲上解析整个 DXF：顺序扫描到 ENTITIES 段前收集块定义，
    /// 再把体量最大的 ENTITIES 段按实体边界切分为多块，用多线程并行展开。
    /// </summary>
    public static List<PathPolyline> ParseBytes(byte[] data)
    {
        var result = new List<PathPolyline>();
        var blocks = new Dictionary<string, BlockDef>(StringComparer.Ordinal);
        // 文本编码按整个文件判定一次：同一文件内所有图层名等字符串用同一编码解码，
        // 避免「GBK 字节恰好构成合法 UTF-8 序列」时逐值判定出错。
        Encoding text = DetectTextEncoding(data);

        var r = new GroupReader(data, 0, data.Length, text, /* index */ 0);
        if (!r.Read()) return result;
        // 标准 DXF 中 BLOCKS 段位于 ENTITIES 段之前：先收集块定义，再在 ENTITIES 段
        // 按 INSERT 的插入点/缩放/旋转展开。ENTITIES 段体量巨大，改为分块并行解析。
        while (true)
        {
            if (r.Code == 0 && r.ValueEquals("SECTION"))
            {
                if (!r.Read()) break;
                if (r.Code == 2 && r.ValueEquals("BLOCKS"))
                {
                    if (!ParseBlocksSection(r, blocks)) break;
                    continue;
                }
                if (r.Code == 2 && r.ValueEquals("ENTITIES"))
                {
                    // r 已停在 ENTITIES 值行之后，即该段实体内容的起始字节
                    int entStart = r.Position;
                    int endsec = IndexOfPattern(data, entStart, EndSecPattern);
                    int entEnd = endsec < 0 ? data.Length : endsec;
                    ParseEntitiesParallel(data, entStart, entEnd, result, blocks, text);
                    // 文件可能包含多个 ENTITIES 段：定位到本段 ENDSEC 行之后继续扫描
                    if (endsec < 0) break;
                    int nlAfter = Array.IndexOf(data, (byte)'\n', endsec, data.Length - endsec);
                    if (nlAfter < 0) break;
                    r = new GroupReader(data, nlAfter + 1, data.Length, text);
                    if (!r.Read()) break;
                    continue; // 以新位置重新检视当前组码
                }
                continue; // 其他段，重新检视当前组码
            }
            if (!r.Read()) break;
        }
        return result;
    }

    private static readonly byte[] EndSecPattern = Encoding.ASCII.GetBytes("ENDSEC");
    private static readonly byte[] CodePagePattern = Encoding.ASCII.GetBytes("$DWGCODEPAGE");

    // 严格 UTF-8（非法序列抛异常），用于全文件校验与解码
    private static readonly Encoding Utf8Strict = new UTF8Encoding(false, throwOnInvalidBytes: true);
    private static readonly Encoding AnsiFallback = CreateAnsiFallback();

    private static Encoding CreateAnsiFallback()
    {
        try
        {
            Encoding.RegisterProvider(CodePagesEncodingProvider.Instance);
            return Encoding.GetEncoding(System.Globalization.CultureInfo.CurrentCulture.TextInfo.ANSICodePage);
        }
        catch
        {
            return Encoding.Latin1;
        }
    }

    /// <summary>
    /// 判定整个文件的文本编码：
    /// 1) HEADER 的 $DWGCODEPAGE 显式声明优先（如 ANSI_936 → GBK）；
    /// 2) 无声明时，纯 ASCII 文件直接用 UTF-8；
    /// 3) 含高位字节时对全文件做严格 UTF-8 校验，任一非法序列即整文件按系统 ANSI 代码页。
    ///    注意不能逐值判定——GBK 双字节对（如「细栅」CF B8 D5 A4）可能恰好构成合法 UTF-8。
    /// </summary>
    private static Encoding DetectTextEncoding(byte[] data)
    {
        // $DWGCODEPAGE 位于 HEADER 段，只在文件头部小范围查找
        int headLen = Math.Min(data.Length, 1 << 20);
        int cpIdx = new ReadOnlySpan<byte>(data, 0, headLen).IndexOf(CodePagePattern);
        if (cpIdx >= 0)
        {
            // 变量名行之后是「3 / ANSI_XXX」组码对，取其后第一个 ANSI_ 值
            int lineEnd = Array.IndexOf(data, (byte)'\n', cpIdx, headLen - cpIdx);
            if (lineEnd > 0)
            {
                var rr = new GroupReader(data, lineEnd + 1, Math.Min(lineEnd + 256, data.Length), Encoding.ASCII);
                if (rr.Read() && rr.Code == 3)
                {
                    string cp = rr.ValueString();
                    if (cp.StartsWith("ANSI_", StringComparison.OrdinalIgnoreCase) &&
                        int.TryParse(cp.AsSpan(5), out int page))
                    {
                        try
                        {
                            Encoding.RegisterProvider(CodePagesEncodingProvider.Instance);
                            return Encoding.GetEncoding(page);
                        }
                        catch { /* 未知代码页，落入启发式判定 */ }
                    }
                    if (cp.Equals("UTF-8", StringComparison.OrdinalIgnoreCase))
                        return Utf8Strict;
                }
            }
        }

        if (!ContainsNonAscii(data)) return Utf8Strict;
        try
        {
            Utf8Strict.GetCharCount(data); // 只校验不产出字符串
            return Utf8Strict;
        }
        catch (DecoderFallbackException)
        {
            return AnsiFallback;
        }
    }

    /// <summary>是否含高位字节（非纯 ASCII）；按 8 字节一组扫描。</summary>
    private static bool ContainsNonAscii(byte[] data)
    {
        var longs = MemoryMarshal.Cast<byte, ulong>(data);
        foreach (ulong v in longs)
            if ((v & 0x8080_8080_8080_8080UL) != 0) return true;
        for (int i = longs.Length * 8; i < data.Length; i++)
            if (data[i] >= 0x80) return true;
        return false;
    }

    /// <summary>在 data[start..] 中查找字节模式，返回绝对偏移；未找到返回 -1。</summary>
    private static int IndexOfPattern(byte[] data, int start, byte[] pattern)
    {
        int idx = new ReadOnlySpan<byte>(data, start, data.Length - start).IndexOf(pattern);
        return idx < 0 ? -1 : start + idx;
    }

    /// <summary>可作为分块起点的顶级实体关键字（VERTEX/SEQEND 等子实体不在其列）。</summary>
    private static readonly string[] TopLevelEntities =
    {
        "INSERT", "LINE", "CIRCLE", "ARC", "LWPOLYLINE", "POLYLINE",
        "ELLIPSE", "SPLINE", "POINT", "TEXT", "MTEXT", "SOLID",
        "3DFACE", "HATCH", "DIMENSION", "LEADER"
    };

    /// <summary>
    /// 将 [entStart,entEnd) 的 ENTITIES 内容按实体边界切成多块并行解析。
    /// 每块起点对齐到某个「0 + 顶级实体」边界，保证任何实体都不会被切断；
    /// 各线程写入独立列表，最后按块序合并以保持稳定顺序。
    /// </summary>
    private static void ParseEntitiesParallel(byte[] data, int entStart, int entEnd,
        List<PathPolyline> result, Dictionary<string, BlockDef> blocks, Encoding text)
    {
        int span = entEnd - entStart;
        int n = Environment.ProcessorCount;
        // 小段直接串行，避免线程调度开销
        if (n <= 1 || span < (4 << 20))
        {
            var r = new GroupReader(data, entStart, entEnd, text);
            ParseEntitiesLoop(r, result, blocks);
            return;
        }

        var bounds = new int[n + 1];
        bounds[0] = entStart;
        bounds[n] = entEnd;
        for (int i = 1; i < n; i++)
        {
            int raw = entStart + (int)((long)span * i / n);
            bounds[i] = AlignBoundary(data, raw, entEnd);
        }
        // 对齐后确保边界单调不减（相邻块可能对齐到同一实体）
        for (int i = 1; i <= n; i++)
            if (bounds[i] < bounds[i - 1]) bounds[i] = bounds[i - 1];

        var locals = new List<PathPolyline>[n];
        Parallel.For(0, n, i =>
        {
            var local = new List<PathPolyline>();
            if (bounds[i] < bounds[i + 1])
            {
                var r = new GroupReader(data, bounds[i], bounds[i + 1], text);
                ParseEntitiesLoop(r, local, blocks);
            }
            locals[i] = local;
        });

        long total = 0;
        for (int i = 0; i < n; i++) total += locals[i].Count;
        if (result.Count + total <= int.MaxValue)
            result.Capacity = (int)(result.Count + total);
        for (int i = 0; i < n; i++) result.AddRange(locals[i]);
    }

    /// <summary>循环分发实体直到 ENDSEC/EOF 或区间耗尽。</summary>
    private static void ParseEntitiesLoop(GroupReader r, List<PathPolyline> res, Dictionary<string, BlockDef> blocks)
    {
        if (!r.Read()) return;
        while (true)
        {
            if (r.Code != 0) { if (!r.Read()) return; continue; }
            if (r.ValueEquals("ENDSEC") || r.ValueEquals("EOF")) return;
            if (!DispatchEntity(r, res, blocks)) return;
        }
    }

    /// <summary>
    /// 从 rawPos 起向后寻找一个「0 + 顶级实体关键字」的行边界，返回该「0」行起始偏移；
    /// 找不到则返回 end。rawPos 可能落在行中间，故先跳到下一行首再逐行判定。
    /// </summary>
    private static int AlignBoundary(byte[] data, int rawPos, int end)
    {
        int pos = rawPos;
        // 跳到下一行首，避免从半行开始误判
        int firstNl = Array.IndexOf(data, (byte)'\n', pos, end - pos);
        if (firstNl < 0) return end;
        pos = firstNl + 1;

        while (pos < end)
        {
            int nl = Array.IndexOf(data, (byte)'\n', pos, end - pos);
            int lineEnd = nl < 0 ? end : nl;
            if (LineEquals(data, pos, lineEnd, "0"))
            {
                int nextStart = nl < 0 ? end : nl + 1;
                if (nextStart < end)
                {
                    int nl2 = Array.IndexOf(data, (byte)'\n', nextStart, end - nextStart);
                    int next2End = nl2 < 0 ? end : nl2;
                    if (IsTopLevelEntity(data, nextStart, next2End))
                        return pos;
                }
            }
            if (nl < 0) break;
            pos = nl + 1;
        }
        return end;
    }

    /// <summary>判定 data[start,lineEnd) 去首尾空白后是否等于给定 ASCII 关键字。</summary>
    private static bool LineEquals(byte[] data, int start, int lineEnd, string ascii)
    {
        int off = start, cnt = lineEnd - start;
        while (cnt > 0 && data[off] <= (byte)' ') { off++; cnt--; }
        while (cnt > 0 && data[off + cnt - 1] <= (byte)' ') cnt--;
        if (cnt != ascii.Length) return false;
        for (int i = 0; i < cnt; i++)
            if (data[off + i] != (byte)ascii[i]) return false;
        return true;
    }

    private static bool IsTopLevelEntity(byte[] data, int start, int lineEnd)
    {
        foreach (var kw in TopLevelEntities)
            if (LineEquals(data, start, lineEnd, kw)) return true;
        return false;
    }

    /// <summary>解析 BLOCKS 段，收集块定义（几何以块局部坐标缓存，展开时再做变换）。</summary>
    private static bool ParseBlocksSection(GroupReader r, Dictionary<string, BlockDef> blocks)
    {
        if (!r.Read()) return false;
        while (true)
        {
            if (r.Code != 0) { if (!r.Read()) return false; continue; }
            if (r.ValueEquals("ENDSEC") || r.ValueEquals("EOF")) return r.Read();
            if (r.ValueEquals("BLOCK"))
            {
                if (!ParseBlock(r, blocks)) return false;
            }
            else if (!SkipBody(r)) return false;
        }
    }

    /// <summary>解析单个 BLOCK...ENDBLK；进入时 r 停在 BLOCK 组码，返回时停在其后的 code==0。</summary>
    private static bool ParseBlock(GroupReader r, Dictionary<string, BlockDef> blocks)
    {
        string name = "";
        double bx = 0, by = 0;
        bool more;
        // BLOCK 头属性：名称(2) 与基点(10/20)
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 2: name = r.ValueString(); break;
                case 10: bx = r.ValueDouble(); break;
                case 20: by = r.ValueDouble(); break;
            }
        }
        var geom = new List<PathPolyline>();
        // 块内实体，直到 ENDBLK（块内嵌套 INSERT 不再递归展开）
        while (more)
        {
            if (r.Code != 0) { more = r.Read(); continue; }
            if (r.ValueEquals("ENDBLK")) { more = SkipBody(r); break; }
            more = DispatchEntity(r, geom, null);
        }
        if (name.Length > 0) blocks[name] = new BlockDef(bx, by, geom);
        return more;
    }

    /// <summary>按实体类型分发；blocks 非空时展开 INSERT。返回是否仍有后续数据。</summary>
    private static bool DispatchEntity(GroupReader r, List<PathPolyline> res, Dictionary<string, BlockDef>? blocks)
    {
        if (r.ValueEquals("LINE")) return AddLine(r, res);
        if (r.ValueEquals("CIRCLE")) return AddCircle(r, res);
        if (r.ValueEquals("ARC")) return AddArc(r, res);
        if (r.ValueEquals("LWPOLYLINE")) return AddLwPolyline(r, res);
        if (r.ValueEquals("ELLIPSE")) return AddEllipse(r, res);
        if (r.ValueEquals("SPLINE")) return AddSpline(r, res);
        if (r.ValueEquals("POLYLINE")) return AddPolyline(r, res);
        if (r.ValueEquals("INSERT")) return AddInsert(r, res, blocks);
        return SkipBody(r);
    }

    /// <summary>读取并丢弃当前实体的属性组码，直到下一个 code==0。</summary>
    private static bool SkipBody(GroupReader r)
    {
        bool more;
        while ((more = r.Read()) && r.Code != 0) { }
        return more;
    }

    // ---------------- 各实体转换 ----------------

    private static bool AddLine(GroupReader r, List<PathPolyline> res)
    {
        double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        string layer = "0";
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 10: x0 = r.ValueDouble(); break;
                case 20: y0 = r.ValueDouble(); break;
                case 11: x1 = r.ValueDouble(); break;
                case 21: y1 = r.ValueDouble(); break;
            }
        }
        var pl = new PathPolyline { Layer = layer };
        pl.Points.Add(new Vec2(x0, y0));
        pl.Points.Add(new Vec2(x1, y1));
        res.Add(pl);
        return more;
    }

    private static bool AddCircle(GroupReader r, List<PathPolyline> res)
    {
        double cx = 0, cy = 0, radius = 0;
        string layer = "0";
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 10: cx = r.ValueDouble(); break;
                case 20: cy = r.ValueDouble(); break;
                case 40: radius = r.ValueDouble(); break;
            }
        }
        if (radius > 0)
        {
            var pl = new PathPolyline { Closed = true, Layer = layer };
            TessellateArc(pl.Points, new Vec2(cx, cy), radius, 0, Math.PI * 2, false);
            res.Add(pl);
        }
        return more;
    }

    private static bool AddArc(GroupReader r, List<PathPolyline> res)
    {
        double cx = 0, cy = 0, radius = 0, deg0 = 0, deg1 = 0;
        string layer = "0";
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 10: cx = r.ValueDouble(); break;
                case 20: cy = r.ValueDouble(); break;
                case 40: radius = r.ValueDouble(); break;
                case 50: deg0 = r.ValueDouble(); break;
                case 51: deg1 = r.ValueDouble(); break;
            }
        }
        if (radius > 0)
        {
            double a0 = deg0 * Math.PI / 180.0;
            double a1 = deg1 * Math.PI / 180.0;
            while (a1 <= a0) a1 += Math.PI * 2;   // DXF 圆弧始终逆时针
            var pl = new PathPolyline { Layer = layer };
            TessellateArc(pl.Points, new Vec2(cx, cy), radius, a0, a1, true);
            res.Add(pl);
        }
        return more;
    }

    private static bool AddLwPolyline(GroupReader r, List<PathPolyline> res)
    {
        var pts = new List<Vec2>();
        var bulges = new List<double>();
        bool closed = false;
        string layer = "0";
        double? curX = null;
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 70: closed = (((int)r.ValueDouble()) & 1) != 0; break;
                case 10: curX = r.ValueDouble(); break;
                case 20:
                    if (curX.HasValue)
                    {
                        pts.Add(new Vec2(curX.Value, r.ValueDouble()));
                        bulges.Add(0);
                        curX = null;
                    }
                    break;
                case 42:
                    if (bulges.Count > 0) bulges[^1] = r.ValueDouble();
                    break;
            }
        }
        BuildBulgePolyline(pts, bulges, closed, layer, res);
        return more;
    }

    private static bool AddPolyline(GroupReader r, List<PathPolyline> res)
    {
        var pts = new List<Vec2>();
        var bulges = new List<double>();
        bool closed = false;
        string layer = "0";
        bool more;
        // 读取 POLYLINE 自身属性
        while ((more = r.Read()) && r.Code != 0)
        {
            if (r.Code == 8) layer = r.ValueStringInterned();
            else if (r.Code == 70) closed = (((int)r.ValueDouble()) & 1) != 0;
        }
        // 读取 VERTEX 序列直到 SEQEND
        while (more)
        {
            if (r.ValueEquals("VERTEX"))
            {
                double vx = 0, vy = 0, vb = 0;
                while ((more = r.Read()) && r.Code != 0)
                {
                    switch (r.Code)
                    {
                        case 10: vx = r.ValueDouble(); break;
                        case 20: vy = r.ValueDouble(); break;
                        case 42: vb = r.ValueDouble(); break;
                    }
                }
                pts.Add(new Vec2(vx, vy));
                bulges.Add(vb);
            }
            else if (r.ValueEquals("SEQEND"))
            {
                while ((more = r.Read()) && r.Code != 0) { }   // 跳过 SEQEND 属性
                break;
            }
            else break;   // 意外实体，结束（r 停在该组码上）
        }
        BuildBulgePolyline(pts, bulges, closed, layer, res);
        return more;
    }

    /// <summary>
    /// 展开 INSERT（块引用）：读取块名(2)、图层(8)、插入点(10/20)、缩放(41/42)、旋转(50)，
    /// 把引用块的每条折线按 (局部点-基点)·缩放 → 旋转 → 平移到插入点 变换后输出。
    /// 图层遵循 DXF 语义：块内 "0" 图层实体继承 INSERT 的图层，其余保留自身图层。
    /// blocks 为 null（如块内嵌套 INSERT）时仅消费组码、不展开。
    /// </summary>
    private static bool AddInsert(GroupReader r, List<PathPolyline> res, Dictionary<string, BlockDef>? blocks)
    {
        string name = "";
        string layer = "0";
        double ix = 0, iy = 0, sx = 1, sy = 1, rotDeg = 0;
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 2: name = r.ValueString(); break;
                case 8: layer = r.ValueStringInterned(); break;
                case 10: ix = r.ValueDouble(); break;
                case 20: iy = r.ValueDouble(); break;
                case 41: sx = r.ValueDouble(); break;
                case 42: sy = r.ValueDouble(); break;
                case 50: rotDeg = r.ValueDouble(); break;
            }
        }
        if (blocks != null && name.Length > 0 && blocks.TryGetValue(name, out var blk))
        {
            double ang = rotDeg * Math.PI / 180.0;
            double ca = Math.Cos(ang), sa = Math.Sin(ang);
            foreach (var src in blk.Geometry)
            {
                var srcPts = src.Points;
                int nPts = srcPts.Count;
                var pl = new PathPolyline
                {
                    Closed = src.Closed,
                    Layer = src.Layer == "0" ? layer : src.Layer
                };
                pl.Points.Capacity = nPts;
                for (int k = 0; k < nPts; k++)
                {
                    Vec2 p = srcPts[k];
                    double lx = (p.X - blk.BaseX) * sx;
                    double ly = (p.Y - blk.BaseY) * sy;
                    pl.Points.Add(new Vec2(ix + lx * ca - ly * sa, iy + lx * sa + ly * ca));
                }
                res.Add(pl);
            }
        }
        return more;
    }

    private static bool AddEllipse(GroupReader r, List<PathPolyline> res)
    {
        double cx = 0, cy = 0, mx = 0, my = 0, ratio = 1, t0 = 0, t1 = Math.PI * 2;
        string layer = "0";
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 10: cx = r.ValueDouble(); break;
                case 20: cy = r.ValueDouble(); break;
                case 11: mx = r.ValueDouble(); break;
                case 21: my = r.ValueDouble(); break;
                case 40: ratio = r.ValueDouble(); break;
                case 41: t0 = r.ValueDouble(); break;
                case 42: t1 = r.ValueDouble(); break;
            }
        }
        var center = new Vec2(cx, cy);
        var major = new Vec2(mx, my);
        if (t1 <= t0) t1 += Math.PI * 2;
        var minor = new Vec2(-major.Y * ratio, major.X * ratio);
        bool closed = Math.Abs((t1 - t0) - Math.PI * 2) < 1e-9;

        double rMax = Math.Max(major.Length, minor.Length);
        int n = ArcSegmentCount(rMax, t1 - t0);
        var pl = new PathPolyline { Closed = closed, Layer = layer };
        int last = closed ? n - 1 : n;
        for (int k = 0; k <= last; k++)
        {
            double t = t0 + (t1 - t0) * k / n;
            pl.Points.Add(center + major * Math.Cos(t) + minor * Math.Sin(t));
        }
        res.Add(pl);
        return more;
    }

    private static bool AddSpline(GroupReader r, List<PathPolyline> res)
    {
        int degree = 3;
        bool closed = false;
        string layer = "0";
        var knots = new List<double>();
        var ctrl = new List<Vec2>();
        var fit = new List<Vec2>();
        double? cx = null, fx = null;
        bool more;
        while ((more = r.Read()) && r.Code != 0)
        {
            switch (r.Code)
            {
                case 8: layer = r.ValueStringInterned(); break;
                case 70: closed = (((int)r.ValueDouble()) & 1) != 0; break;
                case 71: degree = Math.Max(1, (int)r.ValueDouble()); break;
                case 40: knots.Add(r.ValueDouble()); break;
                case 10: cx = r.ValueDouble(); break;
                case 20: if (cx.HasValue) { ctrl.Add(new Vec2(cx.Value, r.ValueDouble())); cx = null; } break;
                case 11: fx = r.ValueDouble(); break;
                case 21: if (fx.HasValue) { fit.Add(new Vec2(fx.Value, r.ValueDouble())); fx = null; } break;
            }
        }

        var pl = new PathPolyline { Closed = closed, Layer = layer };
        if (ctrl.Count > degree && knots.Count == ctrl.Count + degree + 1)
        {
            // 标准 NURBS(权重=1) De Boor 采样
            int samples = Math.Max(ctrl.Count * 12, 64);
            double u0 = knots[degree], u1 = knots[ctrl.Count];
            for (int k = 0; k <= samples; k++)
            {
                double u = u0 + (u1 - u0) * k / samples;
                pl.Points.Add(DeBoor(degree, ctrl, knots, u));
            }
        }
        else if (fit.Count > 1)
        {
            pl.Points.AddRange(fit);
        }
        else if (ctrl.Count > 1)
        {
            pl.Points.AddRange(ctrl);
        }
        if (pl.Points.Count > 1) res.Add(pl);
        return more;
    }

    private static Vec2 DeBoor(int p, List<Vec2> ctrl, List<double> knots, double u)
    {
        int n = ctrl.Count - 1;
        int k = p;
        for (int i = p; i <= n; i++)
        {
            if (u >= knots[i] && u <= knots[i + 1] && knots[i + 1] > knots[i]) { k = i; break; }
            if (i == n) k = n;
        }
        var d = new Vec2[p + 1];
        for (int j = 0; j <= p; j++) d[j] = ctrl[Math.Clamp(j + k - p, 0, n)];
        for (int r = 1; r <= p; r++)
        {
            for (int j = p; j >= r; j--)
            {
                int idx = j + k - p;
                double den = knots[idx + p - r + 1] - knots[idx];
                double alpha = den <= 1e-12 ? 0 : (u - knots[idx]) / den;
                d[j] = d[j - 1] * (1 - alpha) + d[j] * alpha;
            }
        }
        return d[p];
    }

    // ---------------- 弧细分辅助 ----------------

    private static int ArcSegmentCount(double radius, double sweep)
    {
        if (radius < ChordTolerance) return 4;
        double maxStep = 2 * Math.Acos(Math.Max(-1, 1 - ChordTolerance / radius));
        int n = (int)Math.Ceiling(Math.Abs(sweep) / Math.Max(maxStep, 1e-4));
        return Math.Clamp(n, 4, 720);
    }

    private static void TessellateArc(List<Vec2> pts, Vec2 center, double r, double a0, double a1, bool includeEnd)
    {
        int n = ArcSegmentCount(r, a1 - a0);
        int last = includeEnd ? n : n - 1;
        for (int k = 0; k <= last; k++)
        {
            double a = a0 + (a1 - a0) * k / n;
            pts.Add(new Vec2(center.X + r * Math.Cos(a), center.Y + r * Math.Sin(a)));
        }
    }

    /// <summary>把带凸度(bulge)的顶点序列展开为折线</summary>
    private static void BuildBulgePolyline(List<Vec2> pts, List<double> bulges, bool closed, string layer, List<PathPolyline> res)
    {
        if (pts.Count < 2) return;
        var pl = new PathPolyline { Closed = closed, Layer = layer };
        int segCount = closed ? pts.Count : pts.Count - 1;
        for (int i = 0; i < segCount; i++)
        {
            Vec2 p1 = pts[i];
            Vec2 p2 = pts[(i + 1) % pts.Count];
            double bulge = bulges[i];
            pl.Points.Add(p1);
            if (Math.Abs(bulge) > 1e-9)
                AppendBulgeArc(pl.Points, p1, p2, bulge);
        }
        if (!closed) pl.Points.Add(pts[^1]);
        res.Add(pl);
    }

    private static void AppendBulgeArc(List<Vec2> outPts, Vec2 p1, Vec2 p2, double bulge)
    {
        // DXF 凸度规范：bulge = tan(包含角/4)，符号决定弧向与圆心相对弦的位置
        //   bulge > 0：逆时针弧，圆心在弦的左侧（沿 p1→p2 方向看）
        //   bulge < 0：顺时针弧，圆心在弦的右侧
        // normal = (-dir.Y, dir.X) 为弦的左法线，故 center = mid + sign(bulge) * h * normal
        double theta = 4 * Math.Atan(bulge);          // 圆心角（带符号）
        double chord = p1.DistanceTo(p2);
        if (chord < 1e-9) return;
        double r = chord / (2 * Math.Sin(Math.Abs(theta) / 2));
        // 圆心：位于弦的垂直平分线上，符号由 bulge 决定
        Vec2 mid = (p1 + p2) * 0.5;
        double h = r * Math.Cos(Math.Abs(theta) / 2);  // 圆心到弦的距离
        Vec2 dir = (p2 - p1) / chord;
        Vec2 normal = new(-dir.Y, dir.X);
        Vec2 center = bulge > 0 ? mid + normal * h : mid - normal * h;

        double a0 = Math.Atan2(p1.Y - center.Y, p1.X - center.X);
        int n = ArcSegmentCount(r, theta);
        for (int k = 1; k < n; k++)
        {
            double a = a0 + theta * k / n;
            outPts.Add(new Vec2(center.X + r * Math.Cos(a), center.Y + r * Math.Sin(a)));
        }
    }

    /// <summary>块定义：基点 + 以块局部坐标缓存的几何（供 INSERT 引用展开）。</summary>
    private sealed class BlockDef
    {
        public readonly double BaseX;
        public readonly double BaseY;
        public readonly List<PathPolyline> Geometry;
        public BlockDef(double baseX, double baseY, List<PathPolyline> geometry)
        {
            BaseX = baseX;
            BaseY = baseY;
            Geometry = geometry;
        }
    }

    /// <summary>
    /// 前向、零字符串分配的 DXF 组码读取器：在内存字节数组的区间 [start,end) 内
    /// 直接切行、解析组码与数值。数值经 <see cref="Utf8Parser"/> 从 ASCII 字节直接解析，
    /// 避免字符串分配与 GC 压力。整文件已在内存，行不会跨缓冲边界。
    /// </summary>
    private sealed class GroupReader
    {
        private readonly byte[] _data;
        private readonly int _end;
        private readonly Encoding _text;
        private int _pos;

        // 当前组码的值（指向 _data 的一段，仅在下一次 Read 前有效）
        private int _valOff;
        private int _valCnt;

        public int Code;

        public GroupReader(byte[] data, int start, int end, Encoding text) : this(data, start, end, text, 0) { }

        public GroupReader(byte[] data, int start, int end, Encoding text, int indexOffset)
        {
            _data = data;
            _pos = start;
            _end = end;
            _text = text;
            _indexOffset = indexOffset;
        }

        private protected readonly int _indexOffset;
        /// <summary>当前实体的全局索引（从 0 开始）。</summary>
        public int CurrentIndex => _currentIndex + _indexOffset;
        private protected int _currentIndex;

        /// <summary>当前读取位置（下一行的起始字节偏移）。</summary>
        public int Position => _pos;

        /// <summary>读取下一组「组码 / 值」对；返回 false 表示区间结束。</summary>
        public bool Read()
        {
            while (true)
            {
                if (!ReadLine(out int co, out int cn)) return false;
                if (!TryParseInt(_data, co, cn, out Code))
                {
                    // 非法组码行：按对丢弃（消费对应的值行）后继续
                    if (!ReadLine(out _, out _)) return false;
                    continue;
                }
                if (!ReadLine(out _valOff, out _valCnt)) return false;
                return true;
            }
        }

        /// <summary>当前值（去除首尾空白后）是否等于给定 ASCII 关键字。</summary>
        public bool ValueEquals(string ascii)
        {
            int off = _valOff, cnt = _valCnt;
            Trim(_data, ref off, ref cnt);
            if (cnt != ascii.Length) return false;
            for (int i = 0; i < cnt; i++)
                if (_data[off + i] != (byte)ascii[i]) return false;
            return true;
        }

        /// <summary>把当前值解析为 double；解析失败返回 0。</summary>
        public double ValueDouble()
        {
            int off = _valOff, cnt = _valCnt;
            Trim(_data, ref off, ref cnt);
            if (cnt == 0) return 0;
            if (Utf8Parser.TryParse(new ReadOnlySpan<byte>(_data, off, cnt), out double v, out _))
                return v;
            return 0;
        }

        /// <summary>把当前值（去首尾空白）解码为字符串；用 Latin1 保证与原始字节一一对应，便于块名匹配。</summary>
        public string ValueString()
        {
            int off = _valOff, cnt = _valCnt;
            Trim(_data, ref off, ref cnt);
            return cnt == 0 ? string.Empty : Encoding.Latin1.GetString(_data, off, cnt);
        }

        // 字符串驻留缓存：图层名等高重复值避免逐实体分配（每个 reader 独立，无需加锁）
        private readonly List<(byte[] Bytes, string Value)> _internCache = new();

        /// <summary>
        /// 把当前值按文件级判定的编码解码为字符串并驻留：相同字节序列返回同一实例。
        /// 用于图层名等取值集合很小、出现次数极多的字符串。
        /// </summary>
        public string ValueStringInterned()
        {
            int off = _valOff, cnt = _valCnt;
            Trim(_data, ref off, ref cnt);
            if (cnt == 0) return string.Empty;
            var span = new ReadOnlySpan<byte>(_data, off, cnt);
            for (int i = 0; i < _internCache.Count; i++)
                if (span.SequenceEqual(_internCache[i].Bytes)) return _internCache[i].Value;
            string s = _text.GetString(_data, off, cnt);
            _internCache.Add((span.ToArray(), s));
            return s;
        }

        private static bool TryParseInt(byte[] a, int off, int cnt, out int value)
        {
            Trim(a, ref off, ref cnt);
            if (cnt > 0 && Utf8Parser.TryParse(new ReadOnlySpan<byte>(a, off, cnt), out value, out _))
                return true;
            value = 0;
            return false;
        }

        private static void Trim(byte[] a, ref int off, ref int cnt)
        {
            while (cnt > 0 && a[off] <= (byte)' ') { off++; cnt--; }
            while (cnt > 0 && a[off + cnt - 1] <= (byte)' ') cnt--;
        }

        /// <summary>读取一行（不含换行符），输出指向 _data 的一段（off/cnt）。</summary>
        private bool ReadLine(out int off, out int cnt)
        {
            if (_pos >= _end) { off = _pos; cnt = 0; return false; }
            int nl = Array.IndexOf(_data, (byte)'\n', _pos, _end - _pos);
            if (nl < 0)
            {
                off = _pos;
                cnt = _end - _pos;
                if (cnt > 0 && _data[off + cnt - 1] == (byte)'\r') cnt--;
                _pos = _end;
                return true;
            }
            off = _pos;
            cnt = nl - _pos;
            if (cnt > 0 && _data[off + cnt - 1] == (byte)'\r') cnt--;
            _pos = nl + 1;
            return true;
        }
    }
}
