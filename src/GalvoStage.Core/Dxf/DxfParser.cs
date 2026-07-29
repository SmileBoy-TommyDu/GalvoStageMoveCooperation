using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using GalvoStage.Core.Geometry;

namespace GalvoStage.Core.Dxf;

/// <summary>
/// 轻量级 ASCII DXF 解析器：读取 ENTITIES 段中的常见二维实体，
/// 统一细分为折线（PathPolyline），供后续路径规划使用。
/// 支持：LINE / CIRCLE / ARC / LWPOLYLINE(含凸度) / POLYLINE+VERTEX / ELLIPSE / SPLINE
/// </summary>
public static class DxfParser
{
    /// <summary>圆弧细分弦高误差 (mm)</summary>
    public const double ChordTolerance = 0.01;

    private record struct GroupCode(int Code, string Value);

    public static List<PathPolyline> ParseFile(string path)
    {
        using var fs = File.OpenRead(path);
        var encoding = DetectEncoding(fs) ?? Encoding.Default;
        fs.Seek(0, SeekOrigin.Begin);
        using var reader = new StreamReader(fs, encoding, detectEncodingFromByteOrderMarks: false);
        return Parse(reader);
    }

    private static Encoding? DetectEncoding(FileStream fs)
    {
        // 读取前 4 字节检查 BOM
        Span<byte> bom = stackalloc byte[4];
        int read = fs.Read(bom);
        if (read >= 3 && bom[0] == 0xEF && bom[1] == 0xBB && bom[2] == 0xBF)
            return Encoding.UTF8;
        if (read >= 2 && bom[0] == 0xFF && bom[1] == 0xFE)
            return Encoding.Unicode; // UTF-16 LE
        if (read >= 2 && bom[0] == 0xFE && bom[1] == 0xFF)
            return Encoding.BigEndianUnicode; // UTF-16 BE
        if (read >= 4 && bom[0] == 0x00 && bom[1] == 0x00 && bom[2] == 0xFE && bom[3] == 0xFF)
            return Encoding.UTF32; // UTF-32 BE
        if (read >= 4 && bom[0] == 0xFF && bom[1] == 0xFE && bom[2] == 0x00 && bom[3] == 0x00)
            return Encoding.UTF32; // UTF-32 LE

        // 无 BOM：尝试 UTF8（无 BOM），如果结果包含大量替代字符则回退到系统 ANSI（或 GBK）
        fs.Seek(0, SeekOrigin.Begin);
        byte[] all = new byte[fs.Length];
        fs.Read(all, 0, all.Length);
        string utf8 = Encoding.UTF8.GetString(all);
        int replacementCount = 0;
        foreach (char c in utf8)
        {
            if (c == '\uFFFD') replacementCount++;
        }
        // 如果替代字符过多，选择系统默认编码（Windows 下通常是 ANSI/GBK）
        if (replacementCount > 0 && replacementCount * 4 > utf8.Length)
        {
            try
            {
                // 936 为 GBK（针对中文 DXF 的常见编码）
                return Encoding.GetEncoding(936);
            }
            catch
            {
                return Encoding.Default;
            }
        }
        return Encoding.UTF8;
    }

    public static List<PathPolyline> Parse(TextReader reader)
    {
        var codes = ReadGroupCodes(reader);
        var result = new List<PathPolyline>();

        // 遍历全部组码：标准 DXF 只有一个 ENTITIES 段，但部分软件导出的
        // 非标准文件会按图层拆成多个 SECTION/ENTITIES...ENDSEC 段，需逐段解析
        int i = 0;
        while (i < codes.Count - 1)
        {
            if (codes[i].Code == 0 && codes[i].Value == "SECTION" &&
                codes[i + 1].Code == 2 && codes[i + 1].Value == "ENTITIES")
            {
                i = ParseEntitiesSection(codes, i + 2, result);
            }
            else i++;
        }
        return result;
    }

    /// <summary>解析单个 ENTITIES 段，返回段结束后的组码索引</summary>
    private static int ParseEntitiesSection(List<GroupCode> codes, int start, List<PathPolyline> result)
    {
        int i = start;
        while (i < codes.Count)
        {
            if (codes[i].Code != 0) { i++; continue; }
            string type = codes[i].Value;
            if (type == "ENDSEC" || type == "EOF") return i + 1;

            int bodyStart = i + 1;
            int bodyEnd = bodyStart;
            while (bodyEnd < codes.Count && codes[bodyEnd].Code != 0) bodyEnd++;

            switch (type)
            {
                case "LINE": AddLine(codes, bodyStart, bodyEnd, result); break;
                case "CIRCLE": AddCircle(codes, bodyStart, bodyEnd, result); break;
                case "ARC": AddArc(codes, bodyStart, bodyEnd, result); break;
                case "LWPOLYLINE": AddLwPolyline(codes, bodyStart, bodyEnd, result); break;
                case "ELLIPSE": AddEllipse(codes, bodyStart, bodyEnd, result); break;
                case "SPLINE": AddSpline(codes, bodyStart, bodyEnd, result); break;
                case "POLYLINE": bodyEnd = AddPolyline(codes, bodyStart, result); break;
            }
            i = bodyEnd;
        }
        return i;
    }

    private static List<GroupCode> ReadGroupCodes(TextReader reader)
    {
        var list = new List<GroupCode>();
        while (true)
        {
            string? codeLine = reader.ReadLine();
            if (codeLine == null) break;
            string? valueLine = reader.ReadLine();
            if (valueLine == null) break;
            if (int.TryParse(codeLine.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int code))
                list.Add(new GroupCode(code, valueLine.Trim()));
        }
        return list;
    }

    private static double D(string s) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : 0;

    private static Dictionary<int, double> CollectScalars(List<GroupCode> codes, int start, int end)
    {
        var map = new Dictionary<int, double>();
        for (int i = start; i < end; i++)
            map[codes[i].Code] = D(codes[i].Value);   // 同码取最后一个
        return map;
    }

    // ---------------- 各实体转换 ----------------

    private static void AddLine(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        var m = CollectScalars(c, s, e);
        var pl = new PathPolyline();
        pl.Points.Add(new Vec2(m.GetValueOrDefault(10), m.GetValueOrDefault(20)));
        pl.Points.Add(new Vec2(m.GetValueOrDefault(11), m.GetValueOrDefault(21)));
        res.Add(pl);
    }

    private static void AddCircle(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        var m = CollectScalars(c, s, e);
        var center = new Vec2(m.GetValueOrDefault(10), m.GetValueOrDefault(20));
        double r = m.GetValueOrDefault(40);
        if (r <= 0) return;
        var pl = new PathPolyline { Closed = true };
        TessellateArc(pl.Points, center, r, 0, Math.PI * 2, false);
        res.Add(pl);
    }

    private static void AddArc(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        var m = CollectScalars(c, s, e);
        var center = new Vec2(m.GetValueOrDefault(10), m.GetValueOrDefault(20));
        double r = m.GetValueOrDefault(40);
        double a0 = m.GetValueOrDefault(50) * Math.PI / 180.0;
        double a1 = m.GetValueOrDefault(51) * Math.PI / 180.0;
        if (r <= 0) return;
        while (a1 <= a0) a1 += Math.PI * 2;   // DXF 圆弧始终逆时针
        var pl = new PathPolyline();
        TessellateArc(pl.Points, center, r, a0, a1, true);
        res.Add(pl);
    }

    private static void AddLwPolyline(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        var pts = new List<Vec2>();
        var bulges = new List<double>();
        bool closed = false;
        double? curX = null;
        for (int i = s; i < e; i++)
        {
            var (code, value) = (c[i].Code, c[i].Value);
            switch (code)
            {
                case 70: closed = (((int)D(value)) & 1) != 0; break;
                case 10: curX = D(value); break;
                case 20:
                    if (curX.HasValue)
                    {
                        pts.Add(new Vec2(curX.Value, D(value)));
                        bulges.Add(0);
                        curX = null;
                    }
                    break;
                case 42:
                    if (bulges.Count > 0) bulges[^1] = D(value);
                    break;
            }
        }
        BuildBulgePolyline(pts, bulges, closed, res);
    }

    private static int AddPolyline(List<GroupCode> c, int start, List<PathPolyline> res)
    {
        var pts = new List<Vec2>();
        var bulges = new List<double>();
        bool closed = false;
        int i = start;
        // 读取 POLYLINE 自身属性
        while (i < c.Count && c[i].Code != 0)
        {
            if (c[i].Code == 70) closed = (((int)D(c[i].Value)) & 1) != 0;
            i++;
        }
        // 读取 VERTEX 序列直到 SEQEND
        while (i < c.Count)
        {
            if (c[i].Code == 0)
            {
                if (c[i].Value == "SEQEND")
                {
                    i++;
                    while (i < c.Count && c[i].Code != 0) i++;   // 跳过 SEQEND 属性
                    break;
                }
                if (c[i].Value != "VERTEX") break;   // 意外实体，结束
                int vs = i + 1, ve = vs;
                while (ve < c.Count && c[ve].Code != 0) ve++;
                var m = CollectScalars(c, vs, ve);
                pts.Add(new Vec2(m.GetValueOrDefault(10), m.GetValueOrDefault(20)));
                bulges.Add(m.GetValueOrDefault(42));
                i = ve;
            }
            else i++;
        }
        BuildBulgePolyline(pts, bulges, closed, res);
        return i;
    }

    private static void AddEllipse(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        var m = CollectScalars(c, s, e);
        var center = new Vec2(m.GetValueOrDefault(10), m.GetValueOrDefault(20));
        var major = new Vec2(m.GetValueOrDefault(11), m.GetValueOrDefault(21));
        double ratio = m.GetValueOrDefault(40, 1);
        double t0 = m.GetValueOrDefault(41, 0);
        double t1 = m.GetValueOrDefault(42, Math.PI * 2);
        if (t1 <= t0) t1 += Math.PI * 2;
        var minor = new Vec2(-major.Y * ratio, major.X * ratio);
        bool closed = Math.Abs((t1 - t0) - Math.PI * 2) < 1e-9;

        double rMax = Math.Max(major.Length, minor.Length);
        int n = ArcSegmentCount(rMax, t1 - t0);
        var pl = new PathPolyline { Closed = closed };
        int last = closed ? n - 1 : n;
        for (int k = 0; k <= last; k++)
        {
            double t = t0 + (t1 - t0) * k / n;
            pl.Points.Add(center + major * Math.Cos(t) + minor * Math.Sin(t));
        }
        res.Add(pl);
    }

    private static void AddSpline(List<GroupCode> c, int s, int e, List<PathPolyline> res)
    {
        int degree = 3;
        bool closed = false;
        var knots = new List<double>();
        var ctrl = new List<Vec2>();
        var fit = new List<Vec2>();
        double? cx = null, fx = null;
        for (int i = s; i < e; i++)
        {
            var (code, value) = (c[i].Code, c[i].Value);
            switch (code)
            {
                case 70: closed = (((int)D(value)) & 1) != 0; break;
                case 71: degree = Math.Max(1, (int)D(value)); break;
                case 40: knots.Add(D(value)); break;
                case 10: cx = D(value); break;
                case 20: if (cx.HasValue) { ctrl.Add(new Vec2(cx.Value, D(value))); cx = null; } break;
                case 11: fx = D(value); break;
                case 21: if (fx.HasValue) { fit.Add(new Vec2(fx.Value, D(value))); fx = null; } break;
            }
        }

        var pl = new PathPolyline { Closed = closed };
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
    private static void BuildBulgePolyline(List<Vec2> pts, List<double> bulges, bool closed, List<PathPolyline> res)
    {
        if (pts.Count < 2) return;
        var pl = new PathPolyline { Closed = closed };
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
        double theta = 4 * Math.Atan(bulge);          // 圆心角（带符号）
        double chord = p1.DistanceTo(p2);
        if (chord < 1e-9) return;
        double r = chord / (2 * Math.Sin(Math.Abs(theta) / 2));
        // 圆心：位于弦的垂直平分线上
        Vec2 mid = (p1 + p2) * 0.5;
        double h = r * Math.Cos(Math.Abs(theta) / 2);  // 圆心到弦的距离
        Vec2 dir = (p2 - p1) / chord;
        Vec2 normal = new(-dir.Y, dir.X);
        Vec2 center = bulge > 0 ? mid - normal * h : mid + normal * h;

        double a0 = Math.Atan2(p1.Y - center.Y, p1.X - center.X);
        int n = ArcSegmentCount(r, theta);
        for (int k = 1; k < n; k++)
        {
            double a = a0 + theta * k / n;
            outPts.Add(new Vec2(center.X + r * Math.Cos(a), center.Y + r * Math.Sin(a)));
        }
    }

    private static double GetValueOrDefault(this Dictionary<int, double> map, int key, double def = 0)
        => map.TryGetValue(key, out double v) ? v : def;
}
