# UAV Autonomous Exploration System - Detaylı Sistem Rehberi

Bu dosya, UAV otonom keşif sisteminin tüm bileşenlerini, çalıştırma adımlarını ve öğrenilen bilgileri içerir.

---

## 1. Sistem Mimarisi

### 1.1 Genel Veri Akışı

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SENSÖR KATMANI                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  Depth Camera (/camera/points - PointCloud2)                                │
│       │                                                                      │
│       ▼                                                                      │
│  OctoMap Server (uav_navigation)                                            │
│       ├──→ /octomap_binary (3D Voxel Map)                                   │
│       ├──→ /projected_map (2D OccupancyGrid)                                │
│       └──→ /octomap_points (Voxel Centers)                                  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FRONTIER DETECTION                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│  frontier_detector_node                                                      │
│       │ Input: /projected_map                                               │
│       │ Output: /frontier_clusters (FrontierArray)                          │
│       │ Algoritma: BFS clustering + PCA-based splitting                     │
│       ▼                                                                      │
│  viewpoint_generator_node                                                    │
│       │ Input: /frontier_clusters, /octomap_binary, /map                    │
│       │ Output: /frontier_clusters_with_viewpoints (FrontierArray)          │
│       │ Algoritma: Cylindrical sampling, collision check, coverage calc     │
│       ▼                                                                      │
│  frontier_visualizer_node                                                    │
│       │ Output: /frontier_markers (MarkerArray) for RViz                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          EXPLORATION PLANNER                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│  global_tour_planner_node (Greedy Frontier Selector)                        │
│       │ Input: /frontier_clusters_with_viewpoints, /odom                    │
│       │ Output: /exploration/global_tour (ExplorationStatus)                │
│       │ Algoritma: cost = w_dist*dist + w_size*(1/size) + w_angle*angle     │
│       │                  + w_coverage*(1/coverage)                          │
│       ▼                                                                      │
│  nav2_path_planner_node                                                      │
│       │ Input: /exploration/global_tour, /odom                              │
│       │ Output: /exploration/nav2_path (nav_msgs/Path)                      │
│       │ Nav2 global planner kullanarak engelsiz yol planlar                 │
│       ▼                                                                      │
│  path_yaw_injector_node                                                      │
│       │ Input: /exploration/nav2_path                                       │
│       │ Output: /exploration/planned_path (Path with yaw)                   │
│       │ Hedefe 1.5m kala yaw rotasyonu başlatır                             │
│       ▼                                                                      │
│  trajectory_follower_node                                                    │
│       │ Input: /exploration/planned_path, /odom                             │
│       │ Output: /cmd_vel (geometry_msgs/Twist)                              │
│       │ 30Hz PID kontrol, height hold (1.5m)                                │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PX4 BRIDGE                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  cmd_vel_to_px4                                                              │
│       │ Input: /cmd_vel                                                     │
│       │ Output: /fmu/in/trajectory_setpoint (TrajectorySetpoint)            │
│       │ geometry_msgs/Twist → PX4 velocity setpoint dönüşümü                │
│       ▼                                                                      │
│  PX4 SITL → Gazebo Simulation                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Paket Yapısı

| Paket | Konum | Açıklama |
|-------|-------|----------|
| `frontier_exploration` | `src/frontier_exploration/` | Frontier detection ve viewpoint generation |
| `exploration_planner` | `src/exploration_planner/` | Path planning, trajectory following |
| `uav_navigation` | `src/uav_navigation/` | OctoMap server config, 2D projection |
| `cmd_vel_to_px4` | `src/cmd_vel_to_px4/` | ROS2 → PX4 bridge |
| `exploration_metrics` | `src/exploration_metrics/` | Keşif performans ölçümü |
| `px4_msgs` | `src/px4_msgs/` | PX4 mesaj tanımları (submodule) |
| `sim_topic_bridge` | `src/sim_topic_bridge/` | Gazebo-ROS2 topic bridging |

---

## 2. Topic ve Mesaj Listesi

### 2.1 Ana Topics

| Topic | Mesaj Tipi | Publisher | Subscriber | Açıklama |
|-------|-----------|-----------|------------|----------|
| `/camera/points` | PointCloud2 | Gazebo | octomap_server | Depth camera point cloud |
| `/projected_map` | OccupancyGrid | octomap_server | frontier_detector, Nav2 | 2D projected map |
| `/map` | OccupancyGrid | octomap_server | viewpoint_generator | Alias for projected_map |
| `/octomap_binary` | Octomap | octomap_server | viewpoint_generator | 3D voxel map |
| `/odom` | Odometry | PX4/Gazebo | Tüm planner node'ları | UAV state |
| `/frontier_clusters` | FrontierArray | frontier_detector | viewpoint_generator | Raw frontiers |
| `/frontier_clusters_with_viewpoints` | FrontierArray | viewpoint_generator | global_tour_planner | Viewpoints ile frontiers |
| `/exploration/global_tour` | ExplorationStatus | global_tour_planner | nav2_path_planner | Selected target |
| `/exploration/nav2_path` | Path | nav2_path_planner | path_yaw_injector | Raw path from Nav2 |
| `/exploration/planned_path` | Path | path_yaw_injector | trajectory_follower | Path with yaw |
| `/cmd_vel` | Twist | trajectory_follower | cmd_vel_to_px4 | Velocity commands |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | cmd_vel_to_px4 | PX4 | PX4 velocity setpoint |
| `/exploration/metrics` | ExplorationMetrics | exploration_metrics | - | Keşif metrikleri |

### 2.2 Custom Mesajlar

**frontier_exploration/msg/Viewpoint.msg:**
```
geometry_msgs/Point position
float64 yaw
int32 coverage
float64 distance_to_centroid
uint32 cluster_id
int32 cluster_size
geometry_msgs/Point cluster_centroid
```

**frontier_exploration/msg/FrontierCluster.msg:**
```
std_msgs/Header header
uint32 id
uint32 size
geometry_msgs/Point centroid
geometry_msgs/Point[] cells
int32 bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y
float64[2] principal_axis
float64 principal_eigenvalue
frontier_exploration/Viewpoint[] viewpoints
frontier_exploration/ClusterConnection[] connections
std_msgs/ColorRGBA color
```

**exploration_planner/msg/ExplorationStatus.msg:**
```
std_msgs/Header header
uint8 state  # IDLE=0, EXPLORING=1, MOVING=2, REFINING=3, COMPLETED=4
uint32[] cluster_order
geometry_msgs/PoseStamped[] waypoints
geometry_msgs/PoseStamped current_target
uint32 current_waypoint_index
uint32 total_waypoints
float64 estimated_time_remaining
float64 total_distance_remaining
float64 target_coverage
uint32 target_cluster_id
uint32 target_cluster_size
```

**exploration_metrics/msg/ExplorationMetrics.msg:**
```
std_msgs/Header header
float64 exploration_percentage
float64 elapsed_time
int32 gt_known_cells, current_known_cells, matched_cells
float64 occupancy_grid_percentage
int32 gt_known_voxels, current_known_voxels, matched_voxels
float64 octomap_percentage
float64 free_space_ratio, occupied_space_ratio, unknown_space_ratio
float64 iou_free, iou_occupied
float64 exploration_rate, avg_exploration_rate, estimated_completion_time
string comparison_mode
```

---

## 3. Konfigürasyon Dosyaları

### 3.1 Frontier Exploration (`src/frontier_exploration/config/params.yaml`)

```yaml
frontier_detector_node:
  ros__parameters:
    map_topic: "/projected_map"
    min_frontier_size: 5          # Minimum cluster hücre sayısı
    max_cluster_size: 27          # PCA split eşiği
    occupied_threshold: 65        # Occupied kabul eşiği
    pca_split_threshold: 2.0      # Eigenvalue split eşiği

viewpoint_generator_node:
  ros__parameters:
    sensor_range: 3.0             # Sensör menzili (m)
    min_dist: 1.5                 # Min viewpoint mesafesi (m)
    max_dist: 3.0                 # Max viewpoint mesafesi (m)
    num_angle_samples: 16         # Açısal örneklem sayısı
    robot_width: 1.0              # Robot genişliği (m)
    robot_length: 1.0             # Robot uzunluğu (m)
    occlusion_enabled: true       # Occlusion-aware coverage
    use_normal_sampling: true     # Frontier'a bakan viewpoint'ler
```

### 3.2 Exploration Planner (`src/exploration_planner/config/params.yaml`)

```yaml
global_tour_planner_node:
  ros__parameters:
    # Cost ağırlıkları
    w_distance: 1.2               # Mesafe penalty
    w_size: 0.1                   # Küçük cluster penalty
    w_angle: 0.2                  # Yön değişikliği penalty
    w_coverage: 0.3               # Düşük coverage penalty

trajectory_follower_node:
  ros__parameters:
    # Hareket limitleri
    v_max: 2.0                    # Max hız (m/s)
    yaw_rate_max: 2.0             # Max yaw hızı (rad/s)

    # PID kazançları
    kp_linear: 1.2
    kp_yaw: 1.0

    # Yükseklik kontrolü
    target_height: 1.5            # Hedef yükseklik (m)
    height_tolerance: 0.2         # Tolerans (m)

    # Kontrol
    control_rate: 30.0            # Hz
```

### 3.3 Nav2 (`src/exploration_planner/config/nav2_params.yaml`)

```yaml
global_costmap:
  ros__parameters:
    resolution: 0.20
    robot_radius: 0.5
    plugins: ["static_layer", "inflation_layer"]

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
```

### 3.4 Exploration Metrics (`src/exploration_metrics/config/params.yaml`)

```yaml
exploration_metrics_node:
  ros__parameters:
    comparison_mode: "both"       # "occupancy_grid", "octomap", "both"
    ground_truth_octomap_path: ""
    ground_truth_occupancy_grid_path: ""
    publish_rate: 1.0             # Hz
    logging_enabled: true
    log_file_path: "/tmp/exploration_metrics.csv"

map_saver_node:
  ros__parameters:
    save_directory: "/home/batoddy/uav_ws/src/exploration_metrics/ground_truth"
    auto_timestamp: true
```

---

## 4. Çalıştırma Adımları

### 4.1 Ön Gereksinimler

```bash
# Workspace'i source et
cd /home/batoddy/uav_ws
source install/setup.bash

# PX4 SITL başlat (ayrı terminal)
# cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth
```

### 4.2 Tam Sistem Başlatma (5 Terminal)

**Terminal 1 - OctoMap Server:**
```bash
cd /home/batoddy/uav_ws && source install/setup.bash
ros2 launch uav_navigation navigation.launch.py use_sim_time:=true
```

**Terminal 2 - Frontier Detection:**
```bash
cd /home/batoddy/uav_ws && source install/setup.bash
ros2 launch frontier_exploration frontier_exploration.launch.py use_sim_time:=true
```

**Terminal 3 - Exploration Planner:**
```bash
cd /home/batoddy/uav_ws && source install/setup.bash
ros2 launch exploration_planner exploration_planner.launch.py use_sim_time:=true rviz:=true
```

**Terminal 4 - PX4 Bridge:**
```bash
cd /home/batoddy/uav_ws && source install/setup.bash
ros2 launch cmd_vel_to_px4 px4_bridge.launch.py
```

**Terminal 5 - Exploration Metrics (Opsiyonel):**
```bash
cd /home/batoddy/uav_ws && source install/setup.bash
ros2 launch exploration_metrics exploration_metrics.launch.py use_sim_time:=true
```

### 4.3 Keşif Kontrolü

```bash
# Keşfi başlat
ros2 service call /exploration/start std_srvs/srv/Trigger

# Keşfi durdur
ros2 service call /exploration/stop std_srvs/srv/Trigger

# Durumu kontrol et
ros2 topic echo /exploration/global_tour
```

---

## 5. Keşif Metrik Sistemi

### 5.1 Ground Truth Kaydetme

```bash
# 1. Haritayı tam olarak keşfet (manuel veya otomatik)
# 2. Ground truth olarak kaydet
ros2 service call /exploration_metrics/save_ground_truth std_srvs/srv/Trigger

# Sadece OctoMap kaydet
ros2 service call /exploration_metrics/save_octomap std_srvs/srv/Trigger

# Sadece OccupancyGrid kaydet
ros2 service call /exploration_metrics/save_occupancy_grid std_srvs/srv/Trigger
```

### 5.2 Metrik Toplama ile Keşif

```bash
ros2 launch exploration_metrics exploration_metrics.launch.py \
    use_sim_time:=true \
    ground_truth_octomap_path:=/home/batoddy/uav_ws/src/exploration_metrics/ground_truth/ground_truth.bt \
    ground_truth_occupancy_grid_path:=/home/batoddy/uav_ws/src/exploration_metrics/ground_truth/ground_truth.yaml \
    log_file_path:=/tmp/experiment_001.csv
```

### 5.3 Sonuç Analizi

```bash
# Grafik çiz
ros2 run exploration_metrics plot_metrics.py --file /tmp/experiment_001.csv --summary

# Birden fazla deney karşılaştır
ros2 run exploration_metrics plot_metrics.py \
    --files /tmp/exp1.csv /tmp/exp2.csv \
    --labels "Greedy" "TSP" \
    --output comparison.png

# Canlı izleme (PlotJuggler ile)
ros2 run plotjuggler plotjuggler
# /exploration/metrics topic'ini seç
```

---

## 6. Debug ve Monitoring

### 6.1 Topic İzleme

```bash
# Frontier'ları izle
ros2 topic echo /frontier_clusters_with_viewpoints

# Mevcut hedefi izle
ros2 topic echo /exploration/global_tour

# Path'i izle
ros2 topic echo /exploration/planned_path

# Velocity komutlarını izle
ros2 topic echo /cmd_vel

# Metrikleri izle
ros2 topic echo /exploration/metrics
```

### 6.2 TF Tree

```bash
ros2 run tf2_tools view_frames
# frames.pdf oluşturur

ros2 run tf2_ros tf2_echo map base_link
```

### 6.3 RViz Görselleştirme

RViz'de eklenecek display'ler:
- `/frontier_markers` - MarkerArray (Frontier'lar)
- `/exploration/planned_path` - Path (Planlanan yol)
- `/projected_map` - Map (2D harita)
- `/exploration/metrics_markers` - MarkerArray (Keşif progress)

---

## 7. Sık Karşılaşılan Sorunlar

### 7.1 "No frontiers detected"
- **Sebep:** Harita henüz oluşmamış veya tüm frontier'lar keşfedilmiş
- **Çözüm:** OctoMap server'ın /projected_map yayınladığını kontrol et

### 7.2 "Path planning failed"
- **Sebep:** Nav2 hedef noktaya ulaşamıyor (engel var veya costmap yanlış)
- **Çözüm:**
  - Costmap'i RViz'de kontrol et
  - `robot_radius` parametresini düşür
  - `inflation_radius` değerini kontrol et

### 7.3 "Drone yükseklikte sabit kalmıyor"
- **Sebep:** Height controller kazançları yetersiz
- **Çözüm:** `kp_height` değerini artır veya `target_height` değişkenini kontrol et

### 7.4 "Keşif çok yavaş"
- **Sebep:** Cost fonksiyonu ağırlıkları optimal değil
- **Çözüm:**
  - `w_distance` değerini düşür (uzak frontier'lar da seçilebilsin)
  - `w_coverage` değerini artır (yüksek coverage'lı hedefler öncelikli)

---

## 8. Build Komutları

```bash
# Tüm workspace
cd /home/batoddy/uav_ws
colcon build

# Tek paket
colcon build --packages-select exploration_metrics

# Bağımlılıklarla birlikte
colcon build --packages-up-to exploration_planner

# Debug build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Temiz build
rm -rf build/ install/ log/
colcon build
```

---

## 9. Dosya Konumları Özeti

```
/home/batoddy/uav_ws/
├── src/
│   ├── frontier_exploration/
│   │   ├── config/params.yaml
│   │   ├── launch/frontier_exploration.launch.py
│   │   ├── msg/  (Viewpoint, FrontierCluster, FrontierArray)
│   │   └── src/  (frontier_detector, viewpoint_generator, frontier_visualizer)
│   │
│   ├── exploration_planner/
│   │   ├── config/params.yaml, nav2_params.yaml
│   │   ├── launch/exploration_planner.launch.py
│   │   ├── msg/  (ExplorationStatus, Trajectory, TelemetryStatus)
│   │   └── src/  (global_tour_planner, nav2_path_planner, path_yaw_injector, trajectory_follower)
│   │
│   ├── exploration_metrics/
│   │   ├── config/params.yaml
│   │   ├── launch/exploration_metrics.launch.py
│   │   ├── msg/ExplorationMetrics.msg
│   │   ├── src/  (exploration_metrics_node, map_saver_node)
│   │   ├── scripts/plot_metrics.py
│   │   └── ground_truth/  (kaydedilen haritalar)
│   │
│   ├── uav_navigation/
│   │   └── launch/navigation.launch.py  (OctoMap server)
│   │
│   └── cmd_vel_to_px4/
│       └── launch/px4_bridge.launch.py
│
├── CLAUDE.md  (Genel proje bilgisi)
└── SYSTEM_GUIDE.md  (Bu dosya - detaylı rehber)
```

---

## 10. Versiyon Bilgisi

- **ROS2 Distro:** Humble
- **PX4 Version:** v1.16
- **Gazebo:** Gazebo Sim (Fortress/Garden)
- **OctoMap Resolution:** 0.20m
- **Target Flight Height:** 1.5m

---

*Bu dosya Claude Code tarafından otomatik oluşturulmuştur. Son güncelleme: 2026-01-24*
