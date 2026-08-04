# Generate a series of drilling DXF samples for galvo-first strategy benchmarking.
# Outputs to src/GalvoStage.App/Samples/bench-*.dxf
# Each file: holes on a 600x400mm board centered at origin.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$outDir = Join-Path $root 'src\GalvoStage.App\Samples'
if (!(Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }

# Deterministic LCG RNG via global state (simple & reliable)
$script:RngSeed = 0
function RngSeed([long]$s) { $script:RngSeed = $s % 2147483648 }
function RngNext() {
    $script:RngSeed = (1103515245 * $script:RngSeed + 12345) % 2147483648
    return [double]$script:RngSeed / 2147483648.0
}
function RngRange([double]$lo, [double]$hi) { return $lo + ($hi - $lo) * (RngNext) }

function Write-DxfHeader($w) {
    $w.WriteLine("  0"); $w.WriteLine("SECTION")
    $w.WriteLine("  2"); $w.WriteLine("HEADER")
    $w.WriteLine("  9"); $w.WriteLine("`$ACADVER")
    $w.WriteLine("  1"); $w.WriteLine("AC1015")
    $w.WriteLine("  0"); $w.WriteLine("ENDSEC")
    $w.WriteLine("  0"); $w.WriteLine("SECTION")
    $w.WriteLine("  2"); $w.WriteLine("ENTITIES")
}
function Write-DxfFooter($w) {
    $w.WriteLine("  0"); $w.WriteLine("ENDSEC")
    $w.WriteLine("  0"); $w.WriteLine("EOF")
}
function Write-Circle($w, [double]$x, [double]$y, [double]$r, [string]$layer) {
    $w.WriteLine("  0"); $w.WriteLine("CIRCLE")
    $w.WriteLine("  8"); $w.WriteLine($layer)
    $w.WriteLine(" 10"); $w.WriteLine($x.ToString("F4"))
    $w.WriteLine(" 20"); $w.WriteLine($y.ToString("F4"))
    $w.WriteLine(" 40"); $w.WriteLine($r.ToString("F4"))
}

# 1. Small uniform grid: 100 holes (10x10)
$path = Join-Path $outDir 'bench-01-small-uniform.dxf'
$w = [System.IO.StreamWriter]::new($path, $false, [System.Text.Encoding]::ASCII)
Write-DxfHeader $w
for ($j = 0; $j -lt 10; $j++) {
    for ($i = 0; $i -lt 10; $i++) {
        $x = -270 + $i * 60
        $y = -180 + $j * 40
        Write-Circle $w $x $y 0.5 'VIA'
    }
}
Write-DxfFooter $w; $w.Close()
Write-Host "Generated $path (100 holes)"

# 2. Medium uniform grid: 1024 holes (32x32)
$path = Join-Path $outDir 'bench-02-medium-uniform.dxf'
$w = [System.IO.StreamWriter]::new($path, $false, [System.Text.Encoding]::ASCII)
Write-DxfHeader $w
$cols = 32; $rows = 32
for ($j = 0; $j -lt $rows; $j++) {
    for ($i = 0; $i -lt $cols; $i++) {
        $x = -290 + ($i + 0.5) * 600.0 / $cols
        $y = -190 + ($j + 0.5) * 400.0 / $rows
        Write-Circle $w $x $y 0.5 'VIA'
    }
}
Write-DxfFooter $w; $w.Close()
Write-Host "Generated $path ($($cols*$rows) holes)"

# 3. Large random: 10000 holes
$path = Join-Path $outDir 'bench-03-large-random.dxf'
RngSeed 42
$w = [System.IO.StreamWriter]::new($path, $false, [System.Text.Encoding]::ASCII)
Write-DxfHeader $w
for ($k = 0; $k -lt 10000; $k++) {
    $x = RngRange -295 295
    $y = RngRange -195 195
    $d = RngRange 0.3 2.0
    Write-Circle $w $x $y ($d/2) 'MIX'
}
Write-DxfFooter $w; $w.Close()
Write-Host "Generated $path (10000 holes)"

# 4. Extra-large random: 50000 holes
$path = Join-Path $outDir 'bench-04-xlarge-random.dxf'
RngSeed 123
$w = [System.IO.StreamWriter]::new($path, $false, [System.Text.Encoding]::ASCII)
Write-DxfHeader $w
for ($k = 0; $k -lt 50000; $k++) {
    $x = RngRange -295 295
    $y = RngRange -195 195
    $d = RngRange 0.3 1.5
    Write-Circle $w $x $y ($d/2) 'VIA'
}
Write-DxfFooter $w; $w.Close()
Write-Host "Generated $path (50000 holes)"

# 5. Clustered realistic: 100000 holes in 2400 clusters (60x40 grid of 10x10mm cells)
#    Each cluster = 5x5 local grid (2mm spacing) - simulates dense PCB via fields.
$path = Join-Path $outDir 'bench-05-clustered-realistic.dxf'
RngSeed 7
$w = [System.IO.StreamWriter]::new($path, $false, [System.Text.Encoding]::ASCII)
Write-DxfHeader $w
$total = 0
for ($cj = 0; $cj -lt 40; $cj++) {
    for ($ci = 0; $ci -lt 60; $ci++) {
        $ccx = -295 + ($ci + 0.5) * 10
        $ccy = -195 + ($cj + 0.5) * 10
        for ($lj = 0; $lj -lt 5; $lj++) {
            for ($li = 0; $li -lt 5; $li++) {
                $x = $ccx - 4 + $li * 2 + (RngRange -0.2 0.2)
                $y = $ccy - 4 + $lj * 2 + (RngRange -0.2 0.2)
                $d = RngRange 0.3 1.0
                Write-Circle $w $x $y ($d/2) 'VIA'
                $total++
            }
        }
    }
}
Write-DxfFooter $w; $w.Close()
Write-Host "Generated $path ($total holes in 2400 clusters)"

Write-Host ""
Write-Host "All bench DXF files generated in $outDir"
Get-ChildItem $outDir 'bench-*.dxf' | ForEach-Object {
    $sizeMB = $_.Length / 1MB
    Write-Host ("  {0,-45} {1,8:F2} MB" -f $_.Name, $sizeMB)
}
