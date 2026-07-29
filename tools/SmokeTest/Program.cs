using GalvoStage.Core.Dxf;
using GalvoStage.Core.PathPlanning;
using GalvoStage.Core.Simulation;

// 冒烟测试：DXF解析 → 采样 → 频率分解 → 联动仿真（对比补偿开/关）
string dxf = args.Length > 0 ? args[0] : @"..\..\src\GalvoStage.App\Samples\demo.dxf";
var polylines = DxfParser.ParseFile(dxf);
Console.WriteLine($"[DXF] 轮廓数={polylines.Count}  总长={polylines.Sum(p => p.Length):F1} mm");
if (polylines.Count == 0) { Console.WriteLine("解析失败!"); return 1; }

var traj = PathSampler.Sample(polylines, feedSpeed: 80, rapidSpeed: 300, sampleRate: 1000);
Console.WriteLine($"[采样] 点数={traj.Count}  时长={traj.Duration:F2} s");

var plan = FrequencyDecomposer.DecomposeAuto(traj, galvoFov: 5);
Console.WriteLine($"[分解] 截止频率={plan.CutoffHz:F2} Hz  振镜最大偏摆={plan.MaxGalvoDeviation:F3} mm (视场±5)");
Console.WriteLine($"[分解] 平台峰值速度={plan.StageMaxVelocity:F1} mm/s  峰值加速度={plan.StageMaxAcceleration:F0} mm/s²");

foreach (bool comp in new[] { false, true })
{
    var sim = new LinkageSimulator(plan, stageBandwidthHz: 12, stageDamping: 0.85,
        disturbAmp: 0.03, disturbFreq: 7, galvoFov: 5, galvoTimeConst: 0.0003)
    { CompensationEnabled = comp };
    sim.Step(sim.Count);
    Console.WriteLine($"[仿真] 补偿={(comp ? "开" : "关")}  最大落点误差={sim.MaxSpotError * 1000:F1} µm  RMS={sim.RmsSpotError * 1000:F1} µm");
}
return 0;
