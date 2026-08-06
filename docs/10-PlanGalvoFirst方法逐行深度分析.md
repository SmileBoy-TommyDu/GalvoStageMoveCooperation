# 10 - PlanGalvoFirst 方法逐行深度分析

> 分析对象：`src/GalvoStage.Core/Drilling/DrillPlanner.cs` 中的 `PlanGalvoFirst` 方法
> 核心问题：振镜优先路径规划算法的每一步实现细节、复杂度、设计权衡
>
> 本文对照 07 号文档（振镜优先策略评估）与 08 号文档（折线振镜优先），
> 对 `PlanGalvoFirst` 及其调用的三个排序辅助函数做**逐行级**解析。

---

## 一、方法签名与总体结构

```csharp
private static List<Geometry.Drilling.DrillingPattern.Hole> PlanGalvoFirst(
    List<Geometry.Drilling.DrillingPattern.Hole> holes, double galvoFov)
```

| 参数 | 含义 | 典型值 |
|---|---|---|
| `holes` | 输入孔位列表 | 10k ~ 1M 个 |
| `galvoFov` | 振镜半视场 (mm) | ±5.0 mm |

**输出**：重排后的孔位列表（保证簇内走振镜、簇间才动平台）

**核心思想**：空间聚类 → 簇序优化 → 簇内最近邻 + 2-opt

**算法流水线**：
```
holes
  │  ① 全局包围盒
  │  ② 网格尺寸 = 2·FOV
  │  ③ 密度门槛检查（不足则回退）
  │  ④ 计数排序分桶 O(n)
  │  ⑤ 收集非空簇 + 质心
  │  ⑥ 莫顿码排序簇序 O(K log K)
  │  ⑦ 簇内最近邻 + 2-opt O(Σmᵢ²)
  ▼
ordered holes
```

---

## 二、边界处理（第 85-86 行）

```csharp
int n = holes.Count;
if (n <= 1) return new List<Geometry.Drilling.DrillingPattern.Hole>(holes);
```

| 行号 | 作用 | 说明 |
|---|---|---|
| 85 | 缓存孔数 `n` | 避免循环中重复访问 `Count` 属性 |
| 86 | 短路返回 | 0/1 孔无需排序；**深拷贝**避免修改原数据 |

**设计权衡**：返回新列表而非原地排序，保持与 `OrderByZonal`/`OrderByNearestGrid` 一致的语义（纯函数）。

---

## 三、步骤 1：全局包围盒（第 88-97 行）

```csharp
double minX = double.MaxValue, minY = double.MaxValue;
double maxX = double.MinValue, maxY = double.MinValue;
foreach (var h in holes)
{
    if (h.X < minX) minX = h.X;
    if (h.Y < minY) minY = h.Y;
    if (h.X > maxX) maxX = h.X;
    if (h.Y > maxY) maxY = h.Y;
}
```

- **目的**：确定网格划分的空间范围
- **复杂度**：O(n) 时间 + O(1) 空间
- **边界条件**：n≥2 保证 `minX ≤ maxX`（除非所有孔重合，此时 `dimX=dimY=1`）

---

## 四、步骤 2：网格尺寸计算（第 99-103 行）

```csharp
double cellSize = Math.Max(galvoFov > 0 ? 2 * galvoFov : 1.0, 1e-3);
int dimX = Math.Max(1, (int)Math.Ceiling((maxX - minX) / cellSize));
int dimY = Math.Max(1, (int)Math.Ceiling((maxY - minY) / cellSize));
int totalCells = dimX * dimY;
```

| 变量 | 含义 | 典型值 |
|---|---|---|
| `cellSize` | 网格边长 = 振镜**全**视场（2·FOV） | FOV=5mm → 10mm |
| `dimX/dimY` | X/Y 方向网格数 | 600mm 板 / 10mm = 60 |
| `totalCells` | 总网格数 | 60×40 = 2400 |

**关键设计**：
- `cellSize = 2·FOV` 的物理意义：同一网格内任意两点距离 ≤ √2·cellSize ≈ 1.414·2·FOV，振镜可覆盖
- `Math.Max(..., 1e-3)` 防除零
- `Math.Max(1, ...)` 保证至少 1×1 网格

---

## 五、步骤 2.5：密度门槛（第 105-113 行）

```csharp
double density = (double)n / totalCells;
if (density < 4.0)
{
    return n > MaxPointsPerZone
        ? OrderByZonal(holes, MaxPointsPerZone)
        : OrderByNearestGrid(holes);
}
```

### 5.1 为什么需要密度门槛？

- GF 策略在**稀疏数据**上会退化为"网格遍历"（大量空簇 + 单孔簇），路径被打散
- 密度 < 4 孔/单元时，回退到 Z-order 或最近邻（它们在稀疏场景更紧凑）

### 5.2 回退策略选择

| 条件 | 策略 | 原因 |
|---|---|---|
| `n > 5000` | `OrderByZonal` | O(n log n)，大数据更快 |
| `n ≤ 5000` | `OrderByNearestGrid` | O(n·R²)，小数据路径更短 |

### 5.3 阈值 4.0 的来源

经验值，来自 07 号文档基准测试：
- 100/1024 孔稀疏数据：GF 退化 2.5-6× → 必须回退
- 10000+ 孔密集数据：GF 加速 1.7-2.1× → 保持 GF
- 临界点约在 density = 3-5 之间，取 4.0 作为安全阈值

---

## 六、步骤 3：计数排序分桶（第 115-125 行）

```csharp
var cellOf = new int[n];           // 记录"每个孔属于哪个格子"的编号
var cellCount = new int[totalCells]; // 记录"每个格里有多少个孔"
for (int i = 0; i < n; i++)
{
    int cx = Math.Clamp((int)((holes[i].X - minX) / cellSize), 0, dimX - 1);
    int cy = Math.Clamp((int)((holes[i].Y - minY) / cellSize), 0, dimY - 1);
    int cell = cy * dimX + cx;
    cellOf[i] = cell;
    cellCount[cell]++;
}
```

### 6.1 逐行解析

| 行号 | 作用 |
|---|---|
| 116 | `cellOf[i]`：记录第 i 个孔属于哪个 cell |
| 117 | `cellCount[c]`：每个 cell 的孔数（桶大小） |
| 120-121 | 网格坐标：`(x-minX)/cellSize` 向下取整；`Math.Clamp` 处理浮点误差 |
| 122 | 一维 cell id = `cy*dimX + cx`（行主序） |
| 123-124 | 双写：记录归属 + 累加计数 |

### 6.2 复杂度

- **时间**：O(n) 严格线性
- **空间**：O(n + totalCells)

### 6.3 为何用计数排序而非哈希表？

- 网格是规则的，cell id 天然连续
- 计数排序无哈希冲突，O(n) 严格线性
- 缓存友好（连续数组访问）
- 比 `Dictionary<int, List<int>>` 快 5-10×（无装箱、无哈希计算）

---

## 七、步骤 4：收集非空簇（第 127-160 行）

### 7.1 统计簇数 K（第 128-130 行）

```csharp
int K = 0;
for (int c = 0; c < totalCells; c++)
    if (cellCount[c] > 0) K++;
```

- K = 实际非空簇数（≤ totalCells）
- 典型：2400 网格中约 1500 个非空

### 7.2 分配簇数据结构（第 132-137 行）

```csharp
var clusterCellId = new int[K];        // 簇 → cell id
var clusterCx = new double[K];         // 簇质心 X（累加器）
var clusterCy = new double[K];         // 簇质心 Y（累加器）
var clusterMembers = new List<int>[K]; // 簇内孔索引
var cellToCluster = new int[totalCells]; // cellId → clusterId
Array.Fill(cellToCluster, -1);
```

| 数组 | 作用 | 长度 |
|---|---|---|
| `clusterCellId` | 反向映射：簇 → 原 cell | K |
| `clusterCx/Cy` | 质心累加器（后续除以 cnt） | K |
| `clusterMembers` | 簇内孔索引列表 | K |
| `cellToCluster` | cell → 簇映射（-1=空） | totalCells |

### 7.3 建立 cell → cluster 映射（第 139-147 行）

```csharp
int ki = 0;
for (int c = 0; c < totalCells; c++)
{
    if (cellCount[c] == 0) continue;
    cellToCluster[c] = ki;
    clusterCellId[ki] = c;
    clusterMembers[ki] = new List<int>(cellCount[c]);
    ki++;
}
```

- 按 cell id 顺序遍历，给每个非空 cell 分配递增的 cluster id
- `new List<int>(cellCount[c])`：预分配容量，避免扩容

### 7.4 收集成员 + 累加质心（第 148-154 行）

```csharp
for (int i = 0; i < n; i++)
{
    int ci = cellToCluster[cellOf[i]];
    clusterMembers[ci].Add(i);
    clusterCx[ci] += holes[i].X;
    clusterCy[ci] += holes[i].Y;
}
```

- 通过 `cellOf[i]` → `cellToCluster` 两级跳转找到簇 id
- 同时累加坐标（为后续求质心做准备）

### 7.5 计算质心（第 155-160 行）

```csharp
for (int ci = 0; ci < K; ci++)
{
    int cnt = clusterMembers[ci].Count;
    clusterCx[ci] /= cnt;
    clusterCy[ci] /= cnt;
}
```

- 质心 = 成员坐标算术平均
- 用于后续莫顿码编码（代表簇的空间位置）

**步骤 4 总复杂度**：O(n + totalCells)

---

## 八、步骤 5：簇序莫顿码排序（第 162-163 行）

```csharp
var clusterOrder = OrderClustersByMorton(clusterCx, clusterCy, dimX, dimY);
```

### 8.1 `OrderClustersByMorton` 实现（第 196-215 行）

```csharp
private static int[] OrderClustersByMorton(double[] cx, double[] cy, int dimX, int dimY)
{
    int K = cx.Length;
    int bits = 0;
    int temp = Math.Max(dimX, dimY) - 1;
    while (temp > 0) { bits++; temp >>= 1; }
```

**第 200-202 行**：计算莫顿码位宽
- `bits = ⌈log₂(max(dimX, dimY))⌉`
- 例：dimX=60, dimY=40 → bits=6（因为 2⁶=64 ≥ 60）

```csharp
    var codes = new (ulong code, int idx)[K];
    for (int i = 0; i < K; i++)
    {
        int x = Math.Clamp((int)cx[i], 0, dimX - 1);
        int y = Math.Clamp((int)cy[i], 0, dimY - 1);
        codes[i] = (EncodeMorton64(x, y, bits), i);
    }
```

- 质心坐标向下取整到网格单元（用质心所在 cell 代表簇）
- 编码为 64 位莫顿码（支持最大 2³² × 2³² 网格）

```csharp
    Array.Sort(codes, (a, b) => a.code.CompareTo(b.code));
    var order = new int[K];
    for (int i = 0; i < K; i++) order[i] = codes[i].idx;
    return order;
}
```

- 按莫顿码升序排序（Z-order 曲线遍历顺序）
- 返回排序后的簇索引序列

**复杂度**：O(K log K)，K 为簇数

### 8.2 `EncodeMorton64` 实现（第 217-227 行）

```csharp
private static ulong EncodeMorton64(int x, int y, int bits)
{
    ulong result = 0;
    for (int i = 0; i < bits; i++)
    {
        result |= ((ulong)(x & (1 << i)) << (2 * i)) |
                  ((ulong)(y & (1 << i)) << (2 * i + 1));
    }
    return result;
}
```

**位操作原理**：
- 逐位提取 x 和 y 的第 i 位
- x 位放到结果的 2i 位，y 位放到 2i+1 位
- 交错后形成 Z-order 曲线编码

**示例**（bits=3, x=5=101₂, y=3=011₂）：
```
x 位:  1   0   1      (从高位到低位)
y 位:   0   1   1
结果: 1 0 0 1 1 1 = 0b100111 = 39
```

**莫顿码的关键性质**：
- 空间相邻的单元，莫顿码数值接近
- 排序后保证簇访问顺序在空间上连续
- 比 Hilbert 曲线实现简单，效果接近

---

## 九、步骤 6：簇内最近邻 + 2-opt 优化（第 231-243 行）

```csharp
var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
foreach (int ci in clusterOrder)
{
    var members = clusterMembers[ci];
    // 从簇中心出发，贪心走最近未访问孔，得到初始开路径
    var tour = NearestNeighborInCluster(holes, members, clusterCx[ci], clusterCy[ci]);
    // 簇内孔数不超阀值时用 2-opt 优化（簇受 FOV 约束，通常规模很小）
    if (tour.Count <= TwoOptMaxCluster)
        TwoOptImprove(holes, tour);
    foreach (int idx in tour)
        ordered.Add(holes[idx]);
}
```

### 9.1 两阶段簇内排序

| 阶段 | 方法 | 作用 |
|---|---|---|
| 构造 | `NearestNeighborInCluster` | 从簇质心出发贪心最近邻，生成初始开路径 |
| 优化 | `TwoOptImprove` | 2-opt 反转消除交叉边，簇孔数 ≤ `TwoOptMaxCluster`(300) 时启用 |

### 9.2 `NearestNeighborInCluster`（第 248-274 行）

```csharp
private static List<int> NearestNeighborInCluster(
    List<...Hole> holes, List<int> members, double startX, double startY)
{
    int m = members.Count;
    var tour = new List<int>(m);
    var used = new bool[m];
    double px = startX, py = startY;      // 起点 = 簇质心
    for (int step = 0; step < m; step++)
    {
        int bestJ = -1; double bestD2 = double.MaxValue;
        for (int j = 0; j < m; j++)
        {
            if (used[j]) continue;
            int idx = members[j];
            double dx = holes[idx].X - px, dy = holes[idx].Y - py;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; bestJ = j; }
        }
        used[bestJ] = true;
        tour.Add(members[bestJ]);
        px = holes[members[bestJ]].X;
        py = holes[members[bestJ]].Y;
    }
    return tour;
}
```

- 起点为簇质心（减少第一次大跳），贪心走最近未访问孔
- 返回孔索引的**开路径**访问顺序（非闭合回路）
- 复杂度：O(m²)，m 为簇内孔数

### 9.3 `TwoOptImprove`（第 276-309 行）

```csharp
private static void TwoOptImprove(List<...Hole> holes, List<int> tour)
{
    int m = tour.Count;
    if (m < 4) return;
    bool improved = true;
    int maxPasses = 20;
    while (improved && maxPasses-- > 0)
    {
        improved = false;
        for (int i = 0; i < m - 1; i++)
            for (int k = i + 2; k < m; k++)
            {
                int a = tour[i], b = tour[i + 1], c = tour[k];
                double before = Dist(holes, a, b);
                double after  = Dist(holes, a, c);
                if (k + 1 < m)   // 开路径：(c,d) 边仅当 k+1 < m 时存在
                {
                    int d = tour[k + 1];
                    before += Dist(holes, c, d);
                    after  += Dist(holes, b, d);
                }
                if (after + 1e-9 < before)
                {
                    tour.Reverse(i + 1, k - i);
                    improved = true;
                }
            }
    }
}
```

**开路径 2-opt 要点**：
- 反转 `[i+1, k]` 区间以消除边 `(a,b)`/`(c,d)` 的交叉
- **开路径特判**：末端点无回边，`k+1 == m` 时仅比较 `(a,b)` vs `(a,c)`
- `maxPasses=20` 上限防止极端数据反复迭代；`1e-9` 容差防浮点抖动
- 复杂度：O(passes · m²)，故仅对 `m ≤ 300` 的簇启用

### 9.4 复杂度分析

- 外层循环：K 个簇
- 每簇：最近邻 O(m²) + 2-opt O(passes·m²)
- 总复杂度：O(Σmᵢ²) ≈ O(K · m̄²)，m̄ 为平均簇大小
- 典型：K=1500, m̄=40 → 约 240 万次基础操作，2-opt 摊薄后仍在毫秒级

### 9.5 为何不用 KD-Tree 加速？

- 簇内孔数通常 < 100，暴力法更快（无建树开销）
- 代码简洁，缓存友好
- KD-Tree 建树 O(m log m) + 查询 O(m log m)，对小 m 反而更慢

---

## 十、回退策略 A：`OrderByZonal`（第 229-279 行）

```csharp
private static List<Geometry.Drilling.DrillingPattern.Hole> OrderByZonal(
    List<Geometry.Drilling.DrillingPattern.Hole> holes, int maxPerZone = 0)
```

### 10.1 网格分辨率（第 252-255 行）

```csharp
int dim = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(n)));
double cw = width / dim;
double ch = height / dim;
```

- 网格数 = ⌈√n⌉，保证平均每格 ~1 个孔
- 例：n=10000 → dim=100，100×100 网格

### 10.2 莫顿码编码（第 257-268 行）

```csharp
var coded = new MortonCode[n];
for (int i = 0; i < n; i++)
{
    int cx = (int)((holes[i].X - minX) / cw);
    int cy = (int)((holes[i].Y - minY) / ch);
    cx = Math.Clamp(cx, 0, dim - 1);
    cy = Math.Clamp(cy, 0, dim - 1);
    uint cellIndex = EncodeMortonCode(cx, cy, dim);
    coded[i] = new MortonCode { OriginalIndex = i, Code = cellIndex, CellX = cx, CellY = cy };
}
```

- 每个孔计算所在 cell 的莫顿码
- 保留原始索引用于重建

### 10.3 排序 + 重建（第 270-278 行）

```csharp
Array.Sort(coded, (a, b) => a.Code.CompareTo(b.Code));
var ordered = new List<Geometry.Drilling.DrillingPattern.Hole>(n);
for (int i = 0; i < n; i++)
    ordered.Add(holes[coded[i].OriginalIndex]);
return ordered;
```

- 按莫顿码排序 → Z-order 曲线遍历顺序
- 复杂度：O(n log n)

### 10.4 `EncodeMortonCode`（第 286-305 行）

与 `EncodeMorton64` 逻辑相同，但返回 32 位码（用于小网格）

---

## 十一、回退策略 B：`OrderByNearestGrid`（第 315-427 行）

### 11.1 网格建桶（第 336-365 行）

```csharp
int dim = Math.Max(1, (int)Math.Sqrt(m));
double cw = Math.Max((maxX - minX) / dim, 1e-9);
double ch = Math.Max((maxY - minY) / dim, 1e-9);

var starts = new int[dim * dim + 1];
var cellOf = new int[m];
for (int k = 0; k < m; k++)
{
    int cx = Math.Clamp((int)((ex[k] - minX) / cw), 0, dim - 1);
    int cy = Math.Clamp((int)((ey[k] - minY) / ch), 0, dim - 1);
    int cell = cy * dim + cx;
    cellOf[k] = cell;
    starts[cell + 1]++;
}
for (int b = 0; b < dim * dim; b++) starts[b + 1] += starts[b];
var items = new int[m];
var fill = (int[])starts.Clone();
for (int k = 0; k < m; k++) items[fill[cellOf[k]]++] = k;
```

- 计数排序建桶：`starts[c]` 到 `starts[c+1]` 是 cell c 的成员在 `items` 中的区间
- 复杂度：O(n)

### 11.2 环形扩展最近邻（第 373-414 行）

```csharp
for (int step = 0; step < n; step++)
{
    int ccx = Math.Clamp((int)((px - minX) / cw), 0, dim - 1);
    int ccy = Math.Clamp((int)((py - minY) / ch), 0, dim - 1);
    int best = -1;
    double bestD2 = double.MaxValue;
    
    for (int r = 0; r <= 2 * dim; r++)
    {
        if (best >= 0 && r > 0)
        {
            double ringMin = (r - 1) * minCell;
            if (ringMin > 0 && ringMin * ringMin > bestD2) break;
        }
        
        int xlo = ccx - r, xhi = ccx + r, ylo = ccy - r, yhi = ccy + r;
        for (int cy = Math.Max(ylo, 0); cy <= Math.Min(yhi, dim - 1); cy++)
        {
            bool edgeRow = cy == ylo || cy == yhi;
            for (int cx = Math.Max(xlo, 0); cx <= Math.Min(xhi, dim - 1); cx++)
            {
                if (!edgeRow && cx != xlo && cx != xhi) continue;
                
                int cell = cy * dim + cx;
                for (int t = starts[cell]; t < starts[cell + 1]; t++)
                {
                    int k = items[t];
                    if (used[k]) continue;
                    
                    double dx = ex[k] - px, dy = ey[k] - py;
                    double d2 = dx * dx + dy * dy;
                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        best = k;
                    }
                }
            }
        }
        
        if (r >= dim && best >= 0) break;
    }
    
    if (best < 0) break;
    
    int pi = epoly[best];
    used[pi] = true;
    var pick = holes[pi];
    ordered.Add(pick);
    px = pick.X;
    py = pick.Y;
}
```

**核心思想**：**环形扩展搜索**
- 从当前点所在 cell 出发，逐圈向外扩展（r=0,1,2,...）
- 每圈只检查**边界** cell（`edgeRow` 或 `cx==xlo/xhi`），避免重复扫描
- **提前终止**：当前圈最小距离 > 已找到最近距离 → 停止扩展

**复杂度**：
- 平均：O(n · R²)，R 为平均搜索半径（典型 R≈3-5）
- 最坏：O(n³)（极端分布），但实际罕见

---

## 十二、算法对比总结

| 算法 | 时间复杂度 | 空间复杂度 | 适用场景 | 路径质量 |
|---|---|---|---|---|
| **PlanGalvoFirst** | O(n + K log K + Σmᵢ²) | O(n + totalCells) | 密集大数据（density ≥ 4） | ★★★★★（平台动 K 次 + 簇内 2-opt） |
| **OrderByZonal** | O(n log n) | O(n) | 超大数据（n > 5000） | ★★★☆☆（无簇内优化） |
| **OrderByNearestGrid** | O(n · R²) 平均 | O(n + dim²) | 小数据（n ≤ 5000） | ★★★★☆（全局最近邻） |

---

## 十三、关键设计决策

| 决策 | 选择 | 原因 |
|---|---|---|
| 密度门槛 | 4.0 | 经验值，平衡 GF 与回退策略的临界点 |
| cellSize | 2·FOV | 物理约束（振镜覆盖范围） |
| 簇序排序 | 莫顿码 | 比 Hilbert 简单，效果接近，O(K log K) 足够快 |
| 簇内排序 | 最近邻 + 2-opt | 小簇（m≤300）质量优于纯贪心，消除交叉边 |
| 分桶策略 | 计数排序 | O(n) 严格线性，缓存友好 |
| 搜索策略 | 环形扩展 | 避免全局扫描，平均 O(n · R²) |

---

## 十四、边界条件与鲁棒性

| 场景 | 处理方式 |
|---|---|
| n=0/1 | 直接返回（第 86 行） |
| 所有孔重合 | dimX=dimY=1，单簇处理 |
| FOV ≤ 0 | cellSize 回退到 1.0（第 100 行） |
| 密度过低 | 回退到 Z-order/最近邻（第 108-113 行） |
| 浮点误差 | `Math.Clamp` 防止越界（第 120-121 行） |
| 超大网格 | 64 位莫顿码支持 2³² × 2³²（第 218 行） |

---

## 十五、与现有文档的关系

| 文档 | 主题 | 与本文的关系 |
|---|---|---|
| 06 - PCB 钻孔链路可行性评估 | 钻孔单链路端到端 | 本文的前置基础 |
| 07 - 大数据钻孔振镜优先策略 | 振镜优先策略设计与评估 | 本文的算法来源 |
| 08 - 折线路径振镜优先实现 | 折线链路的频域分解 | 对比：离散 vs 连续 |
| 09 - 双模式有机融合 | 混合特征加工方案 | 本文算法在混合模式中的应用 |
| **10 - PlanGalvoFirst 逐行分析** | **振镜优先算法细节** | **本文**：算法层面的深度剖析 |

---

## 十六、结论

`PlanGalvoFirst` 是振镜优先策略的**核心实现**，通过“空间聚类 + 莫顿簇序 + 簇内最近邻 + 2-opt”四步流水线，将平台跳跃次数从 n 次降至 K 次（K=簇数，典型 K≈1500）。

**算法亮点**：
1. **密度门槛**：稀疏数据自动回退，避免退化
2. **计数排序**：O(n) 严格线性，缓存友好
3. **莫顿码簇序**：空间连续性保证
4. **簇内最近邻 + 2-opt**：小簇场景快且消除交叉边，质量优于纯贪心

**适用场景**：密集大数据（density ≥ 4），典型如 PCB 钻孔（10k-1M 孔），平台行程 ↓ 40-86%，加工时间 ↓ 70%+。
