# 📊 VIEWPOINT GENERATOR NODE - DETAYLI KOD AÇIKLAMASI

## 📋 ÖZET

**Node Adı**: `viewpoint_generator`  
**Görev**: Frontier cluster'ları için optimal görüş noktaları (viewpoints) üretir

```
GİRDİ:
  ├─ /frontier_clusters (FrontierArray)     ← Frontier Detector'dan gelen kümeler
  ├─ /map (OccupancyGrid)                   ← 2D harita (collision check)
  └─ /octomap_binary (Octomap)               ← 3D engel haritası (3D collision)

ÇIKTI:
  └─ /frontier_clusters_with_viewpoints     ← Cluster'lar + Viewpoint'lar
```

---

## 🏗️ GENEL MİMARİ

```
┌─────────────────────────────────────────────────────────────────┐
│              ViewpointGeneratorNode                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. INPUT PROCESSING                                           │
│     ├─ clustersCallback()     ← Frontier cluster'ları al       │
│     ├─ mapCallback()          ← 2D map al                      │
│     └─ octomapCallback()      ← 3D octomap al                  │
│                                                                 │
│  2. VIEWPOINT GENERATION                                       │
│     ├─ generateViewpoints()   ← Aday viewpoint'lar oluştur    │
│     │   ├─ Silindrik örnekleme (r, θ)                        │
│     │   ├─ 2D collision check (map)                           │
│     │   ├─ 3D collision check (octomap)                       │
│     │   ├─ Yaw optimizasyonu                                  │
│     │   └─ Coverage hesaplama                                 │
│     │                                                           │
│     └─ stabilizeViewpoints()  ← Zamana bağlı filtreleme      │
│         ├─ Önceki VP takip                                    │
│         ├─ Hysteresis uygulanması                             │
│         └─ Jitter azaltma                                     │
│                                                                 │
│  3. OUTPUT                                                      │
│     └─ Cluster'ları viewpoint'larla yayınla                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📥 GİRDİ VERİLERİ (Inputs)

### 1. **FrontierArray** (`/frontier_clusters`)

Her cluster aşağıdakileri içerir:
- `id`: Küme kimliği
- `centroid`: Küme merkezi (x, y, z)
- `bbox_min_x`, `bbox_max_x`: Sınır kutusu X
- `bbox_min_y`, `bbox_max_y`: Sınır kutusu Y
- `cells`: Küme hücrelerinin listesi
- `principal_axis`: PCA axis
- `principal_eigenvalue`: Uzama derecesi

### 2. **OccupancyGrid** (`/map`)

2D harita:
- `width`, `height`: Harita boyutları (hücre cinsinden)
- `resolution`: 1 hücre = ? meter
- `data[]`: Occupancy değerleri (-1: unknown, 0-100: occupancy)

### 3. **Octomap** (`/octomap_binary`)

3D engel haritası:
- 3D ağaç yapısı
- Her noktanın işgal durumu: empty/occupied/unknown

---

## 📤 ÇIKTI VERİLERİ (Outputs)

### **FrontierArray** (`/frontier_clusters_with_viewpoints`)

Giriş cluster'ları + **viewpoints** alanı doldurulmuş

Her **Viewpoint** şunları içerir:
```cpp
struct Viewpoint {
  Point position        // (x, y, z) dünya koordinatları
  double yaw            // Heading yönü (radyan)
  int coverage          // Görülebilen frontier hücre sayısı
  double distance_to_centroid  // Cluster merkez mesafesi
}
```

---

## 🔧 PARAMETRELER

### **Sensör Parametreleri**
```yaml
sensor_range: 5.0          # Maksimum görme mesafesi [m]
sensor_fov_h: 1.57         # Yatay görüş açısı (~90°) [rad]
```

### **Sampling Parametreleri**
```yaml
min_dist: 1.5              # Minimum viewpoint mesafesi [m]
max_dist: 4.0              # Maksimum viewpoint mesafesi [m]
num_dist_samples: 3        # Radyal örnekleme sayısı (r yönü)
num_angle_samples: 12      # Açısal örnekleme sayısı (θ yönü)
                           # Toplam aday = 3 × 12 = 36
```

### **Robot Footprint**
```yaml
robot_width: 0.5           # Robot genişliği X [m]
robot_length: 0.5          # Robot uzunluğu Y [m]
robot_height: 0.3          # Robot yüksekliği [m]
safety_margin: 0.3         # İlave güvenlik mesafesi [m]
flight_height: 1.5         # Uçuş yüksekliği [m]
height_tolerance: 0.2      # Z yönü tolerans ±[m]
```

### **Stabilizasyon Parametreleri** (YENİ)
```yaml
vp_hysteresis_distance: 1.0      # VP bu mesafe içindeyse değiştirme [m]
vp_hysteresis_coverage: 1.3      # Coverage bu kadar artmalı değişim için
vp_tracking_timeout: 5.0         # Eski VP'yi bu süreden sonra unut [s]
vp_stabilization_enabled: true   # Stabilizasyon açık/kapalı
```

---

## 🎯 ALGORITMA ADIM ADIM

### **AŞAMA 1: VIEWPOINT GENERATION** (`generateViewpoints()`)

```
Input: Cluster, 2D Map, 3D OctoMap
Output: En iyi N viewpoint

┌──────────────────────────────────────────────────────┐
│ STEP 1: Silindrik Örnekleme                         │
├──────────────────────────────────────────────────────┤

  Cluster merkezi etrafında radyal ızgara:
  
    θ = 0°, 30°, 60°, ..., 330°  (12 açı)
    r = 1.5m, 2.75m, 4.0m         (3 mesafe)
    
    VP_pos = (centroid_x + r*cos(θ), 
              centroid_y + r*sin(θ), 
              flight_height)
    
    Toplam aday sayısı: 3 × 12 = 36

┌──────────────────────────────────────────────────────┐
│ STEP 2: 2D Collision Check                          │
├──────────────────────────────────────────────────────┤

  hasFootprintClearance2D():
  
    1. Robot footprint'ini grid'e çevir
       clearance_radius = √(w² + l²)/2 + margin
       cell_radius = clearance_radius / resolution
    
    2. Merkezden etrafındaki tüm hücreleri kontrol et
       for each cell in circle(center, cell_radius):
           if occupied or unknown_near_center:
               REJECT
    
    3. Merkez hücre free mi kontrol et
       if not_free(center_cell):
           REJECT
    
    ❌ Rejected: occupied engelliler veya harita dışı

┌──────────────────────────────────────────────────────┐
│ STEP 3: 3D Collision Check (OctoMap)               │
├──────────────────────────────────────────────────────┤

  hasFootprintClearance3D():
  
    1. Robot 3D bounding box oluştur
       [-half_w, half_w] × [-half_l, half_l] × [-half_h, half_h]
       + safety_margin + height_tolerance
    
    2. Grid örnekleme (resolution kadar)
       for each point (px, py, pz) in bbox:
           octree_node = search(px, py, pz)
           if node.occupied:
               REJECT
    
    ❌ Rejected: 3D engeberliler ile çarpışma

┌──────────────────────────────────────────────────────┐
│ STEP 4: Yaw Optimizasyonu                           │
├──────────────────────────────────────────────────────┤

  optimizeYaw():
  
    for yaw in [0°, 10°, 20°, ..., 350°]:
        coverage = computeCoverage(vp_pos, yaw, cluster)
        if coverage > best_coverage:
            best_yaw = yaw
            best_coverage = coverage

┌──────────────────────────────────────────────────────┐
│ STEP 5: Coverage Hesaplama                         │
├──────────────────────────────────────────────────────┤

  computeCoverage(vp_pos, yaw, cluster):
  
    visible_count = 0
    
    for each frontier cell (gx, gy):
        cell_world = gridToWorld(gx, gy)
        
        # Check 1: Range
        dist = distance(vp_pos, cell_world)
        if dist > sensor_range:
            continue
        
        # Check 2: Yaw angle (FOV)
        angle_to_cell = atan2(cell_y - vp_y, cell_x - vp_x)
        if |angle_to_cell - yaw| > fov/2:
            continue
        
        # Check 3: Line of Sight
        if hasLineOfSight(vp_pos, cell_world):
            visible_count++
    
    return visible_count

┌──────────────────────────────────────────────────────┐
│ STEP 6: En İyi N Seçme                             │
├──────────────────────────────────────────────────────┤

    1. Coverage'a göre sırala (azalan)
    2. Top max_viewpoints seç
    3. İlki "best" viewpoint
```

---

### **AŞAMA 2: TEMPORAL STABILIZATION** (`stabilizeViewpoints()`)

```
Input: Yeni oluşturulan viewpoint'lar
Output: Stabilize edilmiş viewpoint'lar

┌────────────────────────────────────────────────────────────┐
│ Tracking Data Yapısı                                       │
├────────────────────────────────────────────────────────────┤

  struct TrackedViewpoint {
    Viewpoint viewpoint          // Önceki frame'deki VP
    Timestamp last_seen          // Ne zaman seenmiş
    Point cluster_centroid       // Küme nerede idi
  }
  
  unordered_map<cluster_id, TrackedViewpoint>

┌────────────────────────────────────────────────────────────┐
│ Stabilizasyon Karar Ağacı                                 │
├────────────────────────────────────────────────────────────┤

      ┌─ Cluster'ın ID'si tracked_viewpoints'da mı?
      │
      ├─ HAYIR → Yeni cluster, registerTrack & return
      │
      └─ EVET
         │
         ├─ Timeout kontrolü
         │  elapsed > 5.0s?
         │  ├─ EVET → Yeni VP ile başla (reset)
         │  └─ HAYIR → devam
         │
         ├─ Küme hareket kontrolü
         │  centroid_distance > 2 * hysteresis_dist?
         │  ├─ EVET → Yeni VP ile başla (cluster moved)
         │  └─ HAYIR → devam
         │
         └─ Hysteresis kontrol
            │
            ├─ VP distance < 1.0m?
            │  new_coverage < old_coverage * 1.3?
            │  ├─ EVET → Eski VP'yi KOR ✓ (stabilize)
            │  │        Validity check:
            │  │        - 2D clearance OK?
            │  │        - 3D clearance OK?
            │  │
            │  └─ HAYIR → Yeni VP'ye güncelle
            │
            └─ VP çok uzak veya coverage çok daha iyi
               └─ Yeni VP'ye güncelle
```

**Hysteresis Mantığı:**
```
Örnek:
  Old VP: coverage = 50, position = (10, 20)
  New VP: coverage = 52, position = (10.5, 20)
  
  Hysteresis distance: 1.0m
  Hysteresis coverage factor: 1.3
  
  Check 1: VP distance = 0.5m < 1.0m? ✓ EVET
  Check 2: 52 < 50 * 1.3 (65)? ✓ EVET
  
  → Eski VP'yi KOR (jitter önle) ✓
```

---

## 📐 ÖNEMLİ FONKSIYONLAR

### **1. `hasLineOfSight()` - Bresenham Algoritması**

```cpp
bool hasLineOfSight(x1, y1, x2, y2, map):
  
  // Bresenham line drawing algoritması
  // Viewpoint → Frontier cell arasında engel var mı?
  
  Adımlar:
  1. Başlangıç ve bitiş noktalarını grid'e çevir
  2. Hücreler arasında çizgi çek
  3. Her hücre engel mi kontrol et
  4. Bir tane bile engel varsa FALSE
  
  return true  // Çizgi görünebilir
```

### **2. `optimizeYaw()` - Yönü Bul**

```cpp
pair<double, int> optimizeYaw():
  
  for yaw in 36 örnek:
      coverage = computeCoverage(vp, yaw)
      if best:
          best_yaw = yaw
          best_coverage = coverage
  
  return (best_yaw_radians, best_coverage_count)
```

### **3. `cleanupTrackedViewpoints()` - Eski Tracking Sil**

```cpp
Eğer cluster artık görünmüyor:
  Timeout > 5.0s?
  → Tracking ata sil
```

---

## 📊 VERÍ AKIŞI ÖRNEK

```
INPUT: FrontierCluster
────────────────────────────────────────────────────────
  id: 1
  centroid: (50, 50, 0)
  bbox: [45-55, 45-55]
  cells: [45,45], [45,46], ..., [55,55]

↓
STEP 1: Silindrik Sampling
────────────────────────────────────────────────────────
  Candidate 1: (51.5, 50, 1.5) @ θ=0°,  r=1.5m
  Candidate 2: (50, 51.5, 1.5) @ θ=90°, r=1.5m
  Candidate 3: (48.5, 50, 1.5) @ θ=180°, r=1.5m
  ...
  Candidate 36: (51.2, 49.2, 1.5) @ θ=350°, r=4.0m

↓
STEP 2-3: Collision Check
────────────────────────────────────────────────────────
  Candidate 1: ✓ PASS 2D, ✓ PASS 3D
  Candidate 2: ✗ FAIL 2D (muro düşük)
  Candidate 3: ✓ PASS 2D, ✓ PASS 3D
  ...
  
  Valid candidates: 28 / 36

↓
STEP 4-5: Yaw Optimization & Coverage
────────────────────────────────────────────────────────
  Candidate 1:
    yaw=10°:  coverage = 15
    yaw=20°:  coverage = 18  ← BEST
    yaw=30°:  coverage = 12
  → Viewpoint: (51.5, 50, 1.5), yaw=20°, coverage=18
  
  Candidate 3:
    Best coverage = 22
  
  Candidate 7:
    Best coverage = 25  ← BEST OVERALL

↓
STEP 6: Top N Selection
────────────────────────────────────────────────────────
  Sort by coverage:
  1. Candidate 7: coverage=25, pos=(52.1, 48.3, 1.5)
  2. Candidate 15: coverage=23, pos=(48.9, 51.2, 1.5)
  3. Candidate 22: coverage=20, pos=(51.5, 50.0, 1.5)
  4. Candidate 8: coverage=18, pos=(50.2, 52.0, 1.5)
  5. Candidate 11: coverage=16, pos=(49.0, 49.5, 1.5)

↓
STEP 7: Temporal Stabilization (NEW)
────────────────────────────────────────────────────────
  Önceki frame:
    VP: (52.0, 48.3, 1.5), coverage=24
  
  Yeni best VP:
    VP: (52.1, 48.3, 1.5), coverage=25
  
  Distance: 0.1m < 1.0m? ✓
  Coverage: 25 < 24*1.3(31.2)? ✓
  
  → Eski VP'yi KOR (stabilize) ✓

↓
OUTPUT: FrontierArray with Viewpoints
────────────────────────────────────────────────────────
  cluster.viewpoints = [
    { pos: (52.0, 48.3, 1.5), yaw: 0.34, coverage: 24 },
    { pos: (48.9, 51.2, 1.5), yaw: 1.57, coverage: 23 },
    ...
  ]
```

---

## 🔍 ÖNEMLI DETAYLAR

### **1. Clearance Radius Hesaplaması**
```cpp
clearance_radius = sqrt(width² + length²)/2 + safety_margin
                 = sqrt(0.5² + 0.5²)/2 + 0.3
                 = 0.354 + 0.3
                 = 0.654m
```

### **2. Coverage Hesaplaması**
- Sadece **görülebilen** frontier hücrelerini say
- 3 koşul aynı anda sağlanmalı:
  1. Sensor range'i içinde mi?
  2. FOV içinde mi?
  3. Line of sight var mı?

### **3. Line of Sight (Bresenham)**
- Viewpoint ile frontier cell arasında engel kontrol
- Hızlı raster algoritması
- Tek engel bile varsa BLOCK

### **4. 3D Collision Checking**
- OctoMap'i 3D olarak tarıyor
- Çözünürlüğü (resolution) kadar adım atıyor
- Robot footprint + safety margin hesapla

---

## 📈 PERFORMANS METRİKLERİ

```
Örnek Çalışma:
  ─────────────────────────────────────
  Cluster sayısı: 5
  Toplam aday: 5 × 36 = 180
  
  2D rejection: 60 (50-60mm muro)
  3D rejection: 25 (OctoMap engelleri)
  Geçerli: 95 aday
  
  Coverage hesap: 95 × 36 yaw = 3420 hesap
  
  Final viewpoints: 5 cluster × 5 VP = 25 viewpoint
  
  Çalışma süresi: ~15-25ms (5 cluster için)
  
  Stabilizasyon ile jitter: -70% azalış
```

---

## 🎨 VİZYAL OĞRENTİ

```
Sensör Modeli:
           ↑ yaw=0°
          /|\
         / | \
        /  |  \  sensor_fov_h = 90°
       /   |   \
      -----VP-----  sensor_range = 5.0m

Frontier Görünürlüğü:
      
      Viewpoint (1.5, 4.0, 1.5)
              *
             /|\
            / | \ ← FOV = 90°
           /  |  \
          /   |   \
         /    ↓    \
        /     E     \ 
       /    frontier  \
      /________________\
      
      E = frontier hücre
      Koşullar:
      1. distance(VP, E) ≤ 5.0m ✓
      2. angle_diff(VP→E, yaw) ≤ 45° ✓
      3. line_of_sight(VP, E) ✓
```

---

## ✅ KONTROL LİSTESİ

- [x] Silindrik sampling
- [x] 2D collision check (footprint)
- [x] 3D collision check (OctoMap)
- [x] Yaw optimization
- [x] Coverage calculation
- [x] Temporal stabilization (FUEL-inspired)
- [x] Hysteresis filtering
- [x] Jitter reduction
- [x] Line of sight (Bresenham)

---

**Son Güncelleme**: 18 Ocak 2026  
**Node Durumu**: ✅ Aktif ve Optimize Edilmiş
