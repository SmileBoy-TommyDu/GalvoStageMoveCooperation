// GalvoFirstBench: compare Z-order vs galvo-first path planning on a series of drilling DXF samples.
// Metrics: planning time, platform travel distance, galvo travel distance, cluster count.

using System;
using System.Diagnostics;
using System.IO;
using GalvoStage.Core.Drilling;
using GalvoStage.Core.Geometry;

const double GalvoFov = 5.0;   // mm, matches MainViewModel default

// Locate bench DXF files relative to this executable's source tree
var samplesDir = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..",
    "src", "GalvoStage.App", "Samples"));
if (!Directory.Exists(samplesDir))
{
    // Fallback: assume running from tools/GalvoFirstBench/bin/...
    samplesDir = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..",
        "src", "GalvoStage.App", "Samples"));
}
Console.WriteLine($"Samples dir: {samplesDir}");
Console.WriteLine();

var files = new[]
{
    ("bench-01-small-uniform.dxf",      "100 holes, 10x10 uniform grid"),
    ("bench-02-medium-uniform.dxf",     "1024 holes, 32x32 uniform grid"),
    ("bench-03-large-random.dxf",       "10000 holes, uniform random"),
    ("bench-04-xlarge-random.dxf",      "50000 holes, uniform random"),
    ("bench-05-clustered-realistic.dxf","60000 holes in 2400 clusters (realistic PCB)"),
};

// Table header
Console.WriteLine("┌────────────────────────────┬────────┬──────────────┬──────────────┬──────────────┬──────────────┬──────────┐");
Console.WriteLine("│ Test case                  │  Holes │ Z-order plat │ GF plat dist │ GF galvo dist│ Speedup plat │ Clusters │");
Console.WriteLine("│                            │        │    dist (mm) │      (mm)    │      (mm)    │            │          │");
Console.WriteLine("├────────────────────────────┼────────┼──────────────┼──────────────┼──────────────┼──────────────┼──────────┤");

foreach (var (fname, desc) in files)
{
    var path = Path.Combine(samplesDir, fname);
    if (!File.Exists(path))
    {
        Console.WriteLine($"│ {fname,-26} │ MISSING");
        continue;
    }

    // Parse
    var sw = Stopwatch.StartNew();
    var pattern = GalvoStage.Core.Dxf.DrillingDxfParser.ParseFile(path);
    sw.Stop();
    int n = pattern.Holes.Count;

    // Strategy A: Z-order (legacy) — use Plan with galvoFirst=false
    var swA = Stopwatch.StartNew();
    var trajZ = DrillPlanner.Plan(pattern, GalvoFov, galvoFirst: false);
    swA.Stop();
    double zPlatDist = TotalDistance(trajZ);

    // Strategy B: galvo-first — cluster by 2*FOV grid, Morton-ordered clusters, NN within
    var swB = Stopwatch.StartNew();
    var trajGF = DrillPlanner.Plan(pattern, GalvoFov, galvoFirst: true);
    swB.Stop();

    // Under galvo-first, platform moves only at cluster boundaries.
    // We detect cluster boundaries by checking for large jumps (> 2*FOV) between consecutive holes.
    double gfPlatDist = 0, gfGalvoDist = 0;
    int clusterCount = 1;
    var moves = trajGF.Moves;
    for (int i = 0; i < moves.Count; i++)
    {
        if (i == 0) continue;
        double dx = moves[i].Position.X - moves[i - 1].Position.X;
        double dy = moves[i].Position.Y - moves[i - 1].Position.Y;
        double d = Math.Sqrt(dx * dx + dy * dy);
        if (d > 2 * GalvoFov)
        {
            // Platform jump
            gfPlatDist += d;
            clusterCount++;
        }
        else
        {
            // Galvo move within cluster
            gfGalvoDist += d;
        }
    }

    double speedup = zPlatDist / Math.Max(gfPlatDist, 1e-9);

    Console.WriteLine(
        $"│ {desc,-26} │ {n,6:N0} │ {zPlatDist,12:N0} │ {gfPlatDist,12:N0} │ {gfGalvoDist,12:N0} │ {speedup,9:F1}× │ {clusterCount,7:N0} │");
}

Console.WriteLine("└────────────────────────────┴────────┴──────────────┴──────────────┴──────────────┴──────────────┴──────────┘");
Console.WriteLine();
Console.WriteLine("Notes:");
Console.WriteLine($"  • Galvo FOV = ±{GalvoFov:F1} mm (cluster grid cell = {2 * GalvoFov:F1} mm)");
Console.WriteLine("  • 'Z-order plat dist' = total path length under legacy Z-order sort (every jump is a platform move).");
Console.WriteLine("  • 'GF plat dist'     = sum of jumps > 2·FOV under galvo-first (these are platform moves).");
Console.WriteLine("  • 'GF galvo dist'    = sum of moves ≤ 2·FOV (these are handled by galvo, no platform motion).");
Console.WriteLine("  • 'Speedup plat'     = Z-order plat dist / GF plat dist  (higher = bigger win).");
Console.WriteLine("  • 'Clusters'         = number of platform jumps under galvo-first.");

static double TotalDistance(DrillPlanner.DrillingTrajectory t)
{
    double sum = 0;
    for (int i = 1; i < t.Moves.Count; i++)
    {
        double dx = t.Moves[i].Position.X - t.Moves[i - 1].Position.X;
        double dy = t.Moves[i].Position.Y - t.Moves[i - 1].Position.Y;
        sum += Math.Sqrt(dx * dx + dy * dy);
    }
    return sum;
}
