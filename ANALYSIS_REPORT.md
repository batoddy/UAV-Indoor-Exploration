# UAV Exploration System - Kapsamlı Kod Analizi

Bu doküman, `frontier_exploration` ve `exploration_planner` paketlerinin detaylı analizini içerir.

---

## 1. SİSTEM MİMARİSİ

### 1.1 Veri Akışı

```
OccupancyGrid (/projected_map)
        │
        ▼
┌─────────────────────────────────────────────────────────────────┐
│                    frontier_exploration paketi                   │
├─────────────────────────────────────────────────────────────────┤
│  frontier_detector_node                                          │
│    - BFS ile frontier clustering                                 │
│    - PCA tabanlı büyük cluster bölme                            │
│                     │                                            │
│                     ▼                                            │
│  viewpoint_generator_node                                        │
│    - Silindirik örnekleme ile viewpoint üretimi                 │
│    - 2D/3D collision check                                      │
│    - Occlusion-aware coverage hesaplama                         │
│    - Temporal stabilization                                      │
│                     │                                            │
│                     ▼                                            │
│  frontier_visualizer_node  → RViz Markers                       │
└─────────────────────────────────────────────────────────────────┘
        │
        ▼ FrontierArray (frontier_clusters_with_viewpoints)
┌─────────────────────────────────────────────────────────────────┐
│                    exploration_planner paketi                    │
├─────────────────────────────────────────────────────────────────┤
│  global_tour_planner_node (Greedy Selector)                     │
│    - Weighted cost function ile en iyi viewpoint seçimi         │
│                     │                                            │
│                     ▼ ExplorationStatus                         │
│  nav2_path_planner_node                                         │
│    - Nav2 action ile collision-free path                        │
│                     │                                            │
│                     ▼ Path                                      │
│  path_yaw_injector_node                                         │
│    - Path boyunca smooth yaw dağıtımı                          │
│                     │                                            │
│                     ▼ Path (with yaw)                           │
│  trajectory_generator_node                                       │
│    - Path → Trajectory (pose + velocity + time)                 │
│                     │                                            │
│                     ▼ Trajectory                                │
│  trajectory_follower_node (Pure Pursuit)                        │
│    - Path takibi, cmd_vel üretimi                               │
│                     │                                            │
│  telemetry_node                                                  │
│    - Real-time telemetri ve CSV logging                         │
│                                                                  │
│  exploration_visualizer_node                                     │
│    - Path trail, waypoint visualization                         │
└─────────────────────────────────────────────────────────────────┘
        │
        ▼ cmd_vel
    PX4 Bridge → Drone
```

---

## 2. FRONTIER_EXPLORATION PAKETİ ANALİZİ

### 2.1 Kaynak Dosyaları

| Dosya | Satır | Amaç |
|-------|-------|------|
| frontier_detector_node.cpp | 498 | Frontier tespiti ve clustering |
| viewpoint_generator_node.cpp | 996 | Viewpoint üretimi ve optimizasyonu |
| frontier_visualizer_node.cpp | 322 | RViz görselleştirme |
| include/common.hpp | ~80 | Ortak utility fonksiyonlar |

### 2.2 Mesaj Tipleri

| Mesaj | Alanlar |
|-------|---------|
| Viewpoint.msg | position, yaw, coverage, distance_to_centroid, cluster_id, cluster_size, cluster_centroid |
| FrontierCluster.msg | id, size, cells[], centroid, principal_axis, principal_eigenvalue, viewpoints[], color, bbox_* |
| FrontierArray.msg | header, clusters[] |
| ClusterConnection.msg | target_cluster_id, cost, path_length |

### 2.3 Node Detayları

#### 2.3.1 frontier_detector_node

**Parametreler:**
| Parametre | Default | Kullanılıyor | Açıklama |
|-----------|---------|--------------|----------|
| map_topic | "/projected_map" | ✅ | Input map topic |
| min_frontier_size | 5 | ✅ | Minimum cluster boyutu |
| max_cluster_size | 50 | ✅ | Bölme öncesi max boyut |
| free_threshold | 25 | ✅ | Free cell eşiği |
| occupied_threshold | 65 | ✅ | Occupied cell eşiği |
| pca_split_threshold | 2.0 | ✅ | PCA eigenvalue bölme eşiği |
| require_occupied_neighbor | false | ✅ | Occupied komşu zorunluluğu |

**Fonksiyonlar:**
| Fonksiyon | Kullanılıyor | Açıklama |
|-----------|--------------|----------|
| `inBounds()` | ✅ | Grid sınır kontrolü |
| `isOccupiedVal()` | ✅ | Occupancy kontrolü |
| `computePCA()` | ✅ | PCA analizi |
| `splitByPCA()` | ✅ | PCA ile cluster bölme |
| `isFrontierCell()` | ✅ | Frontier cell tespiti |
| `buildFrontierMask()` | ✅ | Tek geçişte frontier mask oluşturma |
| `bfsCluster()` | ✅ | BFS ile clustering |
| `recursiveSplit()` | ✅ | Recursive PCA splitting |
| `toMsg()` | ✅ | ClusterPiece → ROS mesajı |
| `detectFrontiers()` | ✅ | Ana pipeline |
| `mapCallback()` | ✅ | ROS callback |

**Durum:** ✅ Ölü kod yok

---

#### 2.3.2 viewpoint_generator_node

**Parametreler:**
| Parametre | Default (kod) | YAML değeri | Kullanılıyor | Durum |
|-----------|---------------|-------------|--------------|-------|
| sensor_range | 5.0 | 7.0 | ✅ | Normal |
| sensor_fov_h | 1.57 | 1.57 | ✅ | Normal |
| min_dist | 1.5 | 1.0 | ✅ | Normal |
| max_dist | 4.0 | 3.0 | ✅ | Normal |
| num_dist_samples | 3 | 5 | ✅ | Normal |
| num_angle_samples | 12 | 16 | ✅ | Normal |
| min_coverage | 3 | 5 | ✅ | Normal |
| max_viewpoints | 5 | 5 | ✅ | Normal |
| free_threshold | 25 | 25 | ✅ | Normal |
| occupied_threshold | 65 | 65 | ✅ | Normal |
| robot_width | 0.5 | 1.0 | ✅ | Normal |
| robot_length | 0.5 | 1.0 | ✅ | Normal |
| robot_height | 0.3 | 0.3 | ✅ | Normal |
| safety_margin | 0.3 | 0.3 | ✅ | Normal |
| flight_height | 1.5 | 1.5 | ✅ | Normal |
| height_tolerance | 0.2 | 0.5 | ✅ | Normal |
| vp_stabilization_enabled | true | true | ✅ | Normal |
| vp_hysteresis_distance | 1.0 | 1.0 | ✅ | Normal |
| vp_hysteresis_coverage | 1.3 | 1.3 | ✅ | Normal |
| vp_tracking_timeout | 5.0 | 5.0 | ✅ | Normal |
| occlusion_enabled | true | true | ✅ | Normal |
| occlusion_angle_bin_rad | 0.02 | 0.02 | ✅ | Normal |
| yaw_samples | 36 | 36 | ✅ | Normal |
| min_los_frontier_cells | 3 | - | ✅ | YAML'da yok |
| min_coverage_ratio | 0.2 | 0.2 | ✅ | Normal |
| use_normal_sampling | true | false | ✅ | ⚠️ Default uyuşmazlığı |
| normal_angle_range | 2.094 | 3.0 | ✅ | ⚠️ Değer uyuşmazlığı |
| **require_los_to_centroid** | - | true | ❌ | **KULLANILMIYOR** |

**Fonksiyonlar:**
| Fonksiyon | Satır | Kullanılıyor | Açıklama |
|-----------|-------|--------------|----------|
| `dist2()` | 198 | ✅ | Squared distance |
| `dist()` | 205 | ✅ | Euclidean distance |
| `normalizeAngle()` | 211 | ✅ | Açı normalizasyonu |
| `mapCallback()` | 223 | ✅ | Map güncelleme |
| `octomapCallback()` | 228 | ✅ | OctoMap güncelleme |
| `clustersCallback()` | 241 | ✅ | Ana işlem callback |
| `fillClusterContextIntoViewpoint()` | 285 | ✅ | Cluster bilgisi enjeksiyonu |
| `computeFrontierNormal()` | 299 | ✅ | Normal yönü hesaplama |
| `isAngleInNormalRange()` | 363 | ✅ | Normal range kontrolü |
| `generateViewpoints()` | 372 | ✅ | Viewpoint üretimi |
| `stabilizeBestViewpoint()` | 480 | ✅ | Temporal stabilization |
| `updateTrackedViewpoint()` | 534 | ✅ | Tracking güncelleme |
| `isViewpointStillValid()` | 545 | ✅ | Geçerlilik kontrolü |
| `cleanupTrackedViewpoints()` | 555 | ✅ | Eski VP temizleme |
| `hasFootprintClearance2D()` | 578 | ✅ | 2D collision check |
| `hasFootprintClearance3D()` | 634 | ✅ | 3D collision check |
| `optimizeYaw()` | 685 | ✅ | Yaw optimizasyonu |
| **`computeCoverageSimple()`** | **718** | ❌ | **ÖLÜ KOD** |
| `precomputeCellData()` | 760 | ✅ | LOS precomputation |
| `computeCoverageOcclusionAwareFast()` | 807 | ✅ | Hızlı occlusion-aware coverage |
| `computeCoverageSimpleFast()` | 844 | ✅ | Hızlı simple coverage |
| `countFrontierCellsWithLOS()` | 864 | ✅ | LOS sayacı |
| `hasLineOfSight()` | 893 | ✅ | Bresenham LOS |

---

#### 2.3.3 frontier_visualizer_node

**Parametreler:**
| Parametre | Default | Kullanılıyor |
|-----------|---------|--------------|
| input_topic | "frontier_clusters_with_viewpoints" | ✅ |
| marker_topic | "frontier_markers" | ✅ |
| info_topic | "fis_info" | ✅ |

**Durum:** ✅ Ölü kod yok

---

### 2.4 frontier_exploration - Tespit Edilen Sorunlar

#### 2.4.1 ÖLÜ KOD

**1. `computeCoverageSimple()` fonksiyonu (viewpoint_generator_node.cpp:718-744)**
```cpp
// KULLANILMIYOR - precomputation versiyonu (computeCoverageSimpleFast) kullanılıyor
int computeCoverageSimple(const geometry_msgs::msg::Point &vp_pos, double yaw,
                          const frontier_exploration::msg::FrontierCluster &cluster,
                          const nav_msgs::msg::OccupancyGrid &map)
{
    const double range2 = sensor_range_ * sensor_range_;
    const double half_fov = sensor_fov_h_ * 0.5;

    int count = 0;
    for (const auto &cell : cluster.cells)
    {
        // ... yaw sample başına LOS hesaplıyor (YAVAŞ)
    }
    return count;
}
```

**Öneri:** Silinebilir. `computeCoverageSimpleFast()` bu fonksiyonun optimize edilmiş versiyonudur.

#### 2.4.2 KULLANILMAYAN PARAMETRELER

**1. `require_los_to_centroid` (params.yaml:67)**
```yaml
require_los_to_centroid: true  # Reject VP if wall blocks view to centroid
```

Bu parametre YAML'da tanımlı ama kodda hiç declare veya kullanılmıyor.

**Öneri:** YAML'dan silinmeli.

#### 2.4.3 PARAMETRE UYUŞMAZLIKLARI

**1. `use_normal_sampling`**
- Kod default: `true`
- YAML değeri: `false`
- Sorun: ROS2'de YAML değeri öncelikli, ama kod default'u kafa karıştırıcı

**2. `normal_angle_range`**
- Kod default: `2.094` (~120°)
- YAML değeri: `3.0` (~172°)
- Yorum: "120 total, ~2.094 rad" ama değer 3.0
- Sorun: Yorum ile değer tutarsız

**3. `min_los_frontier_cells`**
- Kod default: `3`
- YAML'da: Yok
- Öneri: YAML'a eklenmeli

#### 2.4.4 TÜRKÇE YORUMLAR

1. `frontier_detector_node.cpp:200`: `// Harita dışı = UNKNOWN`
2. `params.yaml:2`: `# Tum node'lar bu dosyadan parametre alir`
3. `params.yaml:69`: `// bunu dene`

**Öneri:** İngilizce'ye çevrilmeli.

---

## 3. EXPLORATION_PLANNER PAKETİ ANALİZİ

### 3.1 Kaynak Dosyaları

| Dosya | Satır | Amaç |
|-------|-------|------|
| global_tour_planner_node.cpp | 439 | Greedy frontier seçimi |
| nav2_path_planner_node.cpp | ~200 | Nav2 path planning interface |
| path_yaw_injector_node.cpp | ~150 | Yaw smoothing |
| trajectory_generator_node.cpp | ~200 | Path → Trajectory |
| trajectory_follower_node.cpp | 458 | Pure Pursuit controller |
| exploration_visualizer_node.cpp | ~300 | RViz visualization |
| telemetry_node.cpp | ~250 | Telemetri ve logging |
| include/common.hpp | ~100 | Ortak utilities |

### 3.2 Mesaj Tipleri

| Mesaj | Alanlar |
|-------|---------|
| ExplorationStatus.msg | header, state, waypoints[], current_target, cluster_order[], estimated_time_remaining, total_distance_remaining, target_coverage, target_cluster_id, target_cluster_size |
| TelemetryStatus.msg | header, position, velocity, yaw, state, path_distance, exploration_time, etc. |
| Trajectory.msg | header, poses[], velocities[], times[] |

### 3.3 Node Detayları

#### 3.3.1 global_tour_planner_node (Greedy Frontier Selector)

**Parametreler:**
| Parametre | Default | YAML değeri | Kullanılıyor | Durum |
|-----------|---------|-------------|--------------|-------|
| input_topic | "frontier_clusters_with_viewpoints" | ✅ | ✅ | Normal |
| odom_topic | "/odom" | ✅ | ✅ | Normal |
| output_topic | "/exploration/global_tour" | ✅ | ✅ | Normal |
| costmap_topic | "/global_costmap/costmap" | ✅ | ⚠️ | **Subscribe ama kullanılmıyor** |
| w_distance | 1.0 | 2.0 | ✅ | Normal |
| w_size | 0.3 | 0.02 | ✅ | Normal |
| w_angle | 0.5 | 0.4 | ✅ | Normal |
| w_coverage | 0.6 | 0.4 | ✅ | Normal |
| max_viewpoints_to_evaluate | 5 | 5 | ✅ | Normal |
| v_max | 4.0 | 2.0 | ✅ | Normal |
| yaw_rate_max | 2.5 | 1.5 | ✅ | Normal |
| nav2_enable | false | - | ✅ | **Her zaman false** |

**Fonksiyonlar:**
| Fonksiyon | Kullanılıyor | Açıklama |
|-----------|--------------|----------|
| `startCallback()` | ✅ | Exploration başlatma |
| `stopCallback()` | ✅ | Exploration durdurma |
| `odomCallback()` | ✅ | Odometry güncelleme |
| `poseCallback()` | ⚠️ | **Nadiren kullanılıyor** (fallback) |
| `clustersCallback()` | ✅ | Ana işlem callback |
| `selectBestViewpoint()` | ✅ | En iyi viewpoint seçimi |

#### 3.3.2 nav2_path_planner_node

**Parametreler:**
| Parametre | Default | Kullanılıyor |
|-----------|---------|--------------|
| planner_id | "GridBased" | ✅ |
| goal_frame | "map" | ✅ |
| replan_rate | 2.0 | ✅ |

**Durum:** ✅ Ölü kod yok

#### 3.3.3 path_yaw_injector_node

**Parametreler:**
| Parametre | Default | YAML değeri | Kullanılıyor |
|-----------|---------|-------------|--------------|
| yaw_initial_blend_dist | 1.0 | 1.0 | ✅ |
| yaw_final_blend_dist | 2.5 | 2.5 | ✅ |

**Durum:** ✅ Ölü kod yok

#### 3.3.4 trajectory_generator_node

**Parametreler:**
| Parametre | Default | Kullanılıyor |
|-----------|---------|--------------|
| v_max | 2.5 | ✅ |
| a_max | 0.7 | ✅ |
| yaw_blend_distance | 1.5 | ✅ |

**Durum:** ✅ Ölü kod yok

#### 3.3.5 trajectory_follower_node

**Parametreler (hepsi kullanılıyor):**
| Parametre | Default | YAML |
|-----------|---------|------|
| control_rate | 30.0 | 30.0 |
| goal_tolerance_xy | 0.25 | 0.15 |
| goal_tolerance_yaw | 0.15 | 0.1 |
| kp_linear | 1.0 | 1.0 |
| kd_linear | 0.3 | 0.6 |
| kp_yaw | 1.5 | 1.0 |
| kd_yaw | 0.2 | 0.5 |
| kp_z | 1.0 | 1.0 |
| v_max | 2.5 | 2.0 |
| yaw_rate_max | 1.5 | 1.5 |
| vz_max | 1.0 | 1.0 |
| a_max | 0.8 | 0.5 |
| yaw_accel_max | 1.0 | 0.8 |
| velocity_smoothing | 0.2 | 0.25 |
| slowdown_distance | 1.5 | 1.0 |
| min_approach_speed | 0.3 | 0.4 |
| target_height | 1.5 | 1.5 |
| height_tolerance | 0.2 | 0.2 |
| lookahead_distance | 1.0 | 1.0 |
| min_lookahead_distance | 0.5 | 0.2 |
| lookahead_gain | 0.5 | 0.4 |

**Durum:** ✅ Tüm parametreler kullanılıyor

#### 3.3.6 telemetry_node

**Parametreler:**
| Parametre | Default | YAML | Kullanılıyor |
|-----------|---------|------|--------------|
| publish_rate | 30.0 | 30.0 | ✅ |
| logging_enabled | false | false | ✅ |
| log_file_path | "/tmp/exploration_telemetry.csv" | ✅ | ✅ |

**Durum:** ✅ Ölü kod yok

#### 3.3.7 exploration_visualizer_node

**Parametreler:**
| Parametre | Default | YAML | Kullanılıyor |
|-----------|---------|------|--------------|
| robot_width | 0.5 | 1.0 | ✅ |
| robot_length | 0.5 | 1.0 | ✅ |
| robot_height | 0.2 | 0.3 | ✅ |
| trail_enabled | true | true | ✅ |
| trail_max_points | 5000 | 5000 | ✅ |
| trail_min_distance | 0.1 | 0.1 | ✅ |

**Durum:** ✅ Ölü kod yok

---

### 3.4 exploration_planner - Tespit Edilen Sorunlar

#### 3.4.1 ÖLÜ KOD / KULLANILMAYAN ALANLAR

**1. `costmap_` subscriber (global_tour_planner_node.cpp:119-126)**
```cpp
costmap_sub_ =
    this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
      {
        costmap_ = msg;  // Saklanıyor ama hiç kullanılmıyor!
      });
```

**Sorun:** `costmap_` değişkeni güncelleniyor ama kodda hiçbir yerde kullanılmıyor.

**Öneri:** Subscriber ve ilgili değişkenler silinebilir.

**2. `poseCallback()` (global_tour_planner_node.cpp:186-192)**
```cpp
void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!have_pose_) {  // Sadece odom yoksa çalışır
    current_pose_ = *msg;
    have_pose_ = true;
  }
}
```

**Sorun:** Bu callback sadece ilk pose için fallback. `odomCallback` varken neredeyse hiç çağrılmaz.

**Değerlendirme:** Mavros ile entegrasyon için tutulabilir, ama gereksiz ise silinebilir.

**3. `nav2_enable_` parametresi (global_tour_planner_node.cpp:66)**
```cpp
declare_parameter("nav2_enable", false);
```

**Sorun:** Her zaman `false` olarak kalıyor, hiç aktif edilmiyor. Nav2 goal publishing kodu var ama hiç çalışmıyor.

**Öneri:** Ya aktif edilmeli ya da ilgili kod silinmeli.

#### 3.4.2 KULLANILMAYAN YAML PARAMETRELERİ

params.yaml'daki şu parametreler kodda kullanılmıyor:

| Parametre | Satır | Durum |
|-----------|-------|-------|
| `refine_radius` | 109 | ❌ Kullanılmıyor |
| `max_refine_clusters` | 110 | ❌ Kullanılmıyor |
| `w_consistency` | 111 | ❌ Kullanılmıyor |
| `traj_dt` | 116 | ❌ Kullanılmıyor |
| `lookahead_time` | 117 | ❌ Kullanılmıyor |
| `min_waypoint_dist` | 118 | ❌ Kullanılmıyor |
| `planner_type` | 73 | ❌ Kullanılmıyor (kendi planner'ı yok) |
| `obstacle_threshold` | 74 | ❌ Kullanılmıyor |
| `path_simplify` | 75 | ❌ Kullanılmıyor |
| `astar_allow_diagonal` | 78 | ❌ Kullanılmıyor |
| `rrt_max_iterations` | 81 | ❌ Kullanılmıyor |
| `rrt_step_size` | 82 | ❌ Kullanılmıyor |
| `rrt_goal_bias` | 83 | ❌ Kullanılmıyor |
| `rrt_goal_threshold` | 84 | ❌ Kullanılmıyor |
| `min_height` | 41 | ❌ Kullanılmıyor |
| `max_height` | 42 | ❌ Kullanılmıyor |
| `costmap_resolution` | 43 | ❌ Kullanılmıyor |
| `inflation_radius` | 44 | ❌ Kullanılmıyor |
| `costmap_frame` | 45 | ❌ Kullanılmıyor |
| `map_size_x` | 46 | ❌ Kullanılmıyor |
| `map_size_y` | 47 | ❌ Kullanılmıyor |
| `lethal_cost` | 50 | ❌ Kullanılmıyor |
| `inscribed_cost` | 51 | ❌ Kullanılmıyor |
| `decay_factor` | 52 | ❌ Kullanılmıyor |
| `unknown_cost` | 66 | ❌ Kullanılmıyor |
| `unknown_inflation_radius` | 67 | ❌ Kullanılmıyor |
| `treat_unknown_as_obstacle` | 68 | ❌ Kullanılmıyor |

**Açıklama:** Bu parametreler muhtemelen önceki bir custom path planner implementasyonundan kalmış. Şu an Nav2 kullanıldığı için gereksiz.

---

## 4. ÖZET: TEMİZLENECEKLER

### 4.1 SİLİNECEK KOD

| Dosya | Öğe | Satır | Açıklama |
|-------|-----|-------|----------|
| viewpoint_generator_node.cpp | `computeCoverageSimple()` | 718-744 | Kullanılmayan fonksiyon |
| global_tour_planner_node.cpp | `costmap_sub_` + `costmap_` | 119-126, 417-418 | Kullanılmayan subscriber |

### 4.2 SİLİNECEK PARAMETRELER

**frontier_exploration/config/params.yaml:**
- `require_los_to_centroid: true` (satır 67)

**exploration_planner/config/params.yaml (büyük temizlik):**
- Costmap Generator bölümü (satır 36-68) - tamamı kullanılmıyor
- Path Planner bölümü (satır 71-84) - tamamı kullanılmıyor
- Local Refiner bölümü (satır 107-111) - tamamı kullanılmıyor
- Trajectory Generator'dan `traj_dt`, `lookahead_time`, `min_waypoint_dist`

### 4.3 DÜZELTİLECEK UYUŞMAZLIKLAR

| Dosya | Parametre | Mevcut | Önerilen |
|-------|-----------|--------|----------|
| frontier params.yaml | `use_normal_sampling` | false | Kod default ile uyumlu olmalı |
| frontier params.yaml | `normal_angle_range` | 3.0 | 2.094 (yorum ile uyumlu) |
| frontier params.yaml | `min_los_frontier_cells` | yok | 3 eklenmeli |

### 4.4 TÜRKÇE → İNGİLİZCE ÇEVİRİ

| Dosya | Satır | Mevcut | Önerilen |
|-------|-------|--------|----------|
| frontier_detector_node.cpp | 200 | "Harita dışı = UNKNOWN" | "Out of map bounds = UNKNOWN" |
| frontier params.yaml | 2 | "Tum node'lar bu dosyadan..." | "All nodes read parameters from this file" |
| frontier params.yaml | 69 | "// bunu dene" | "// try this" veya silinmeli |
| exploration params.yaml | 4-5 | Türkçe açıklamalar | İngilizce |

---

## 5. DEĞERLENDİRME: TUTULMASI GEREKENLER

Bazı "kullanılmayan" görünen kodlar aslında gerekli olabilir:

1. **`poseCallback()` (global_tour_planner_node)**: Mavros fallback için tutulabilir
2. **`nav2_enable_` parametresi**: Gelecekte Nav2 entegrasyonu için tutulabilir
3. **YAML'daki costmap/planner parametreleri**: Eğer gelecekte custom planner yazılacaksa tutulabilir

---

## 6. SONRAKI ADIMLAR

1. **Öncelik 1:** `computeCoverageSimple()` fonksiyonunu sil
2. **Öncelik 2:** `costmap_sub_` ve `costmap_` değişkenlerini sil
3. **Öncelik 3:** `require_los_to_centroid` parametresini YAML'dan sil
4. **Öncelik 4:** Parametre uyuşmazlıklarını düzelt
5. **Öncelik 5:** Türkçe yorumları İngilizce'ye çevir
6. **Öncelik 6:** Kullanılmayan YAML parametrelerini temizle (dikkatli ol, gelecek için gerekebilir)

---

*Bu analiz 2026-01-27 tarihinde yapılmıştır.*
