# 🔧 NAV2 COSTMAP QoS FIX - ÖZET

## 🐛 PROBLEM
- Nav2 global costmap Rviz'de görünmüyor
- Terminal output'ta: **"Publisher count: 0"**
- OctoMap çalışıyor, ama costmap yayınlanmıyor

## 🔍 ROOT CAUSE - QoS UYUMSUZLUĞU

| Taraf | Reliability | Durability |
|-------|-------------|-----------|
| **Nav2 Publisher** | Best Effort | Volatile |
| **RViz Subscriber** (eski) | Reliable | Transient Local |
| **❌ UYUMSUZ** | ❌ | ❌ |

Nav2 costmap publisher'ları varsayılan olarak **Best Effort + Volatile** QoS kullanıyor.
RViz **Reliable + Transient Local** bekliyor → **HABERLEŞME YATAĞAMIYOR!**

## ✅ ÇÖZÜM: RViz'i Nav2'ye uydurmak

### Adım 1: Rviz Config Güncelle
**Dosya:** `exploration_planner/config/exploration.rviz`

```yaml
- Class: rviz_default_plugins/Map
  Name: Nav2 Global Costmap
  Topic:
    Value: /planner_server/global_costmap/costmap
    Durability Policy: Volatile        # ← Değişti
    Reliability Policy: Best Effort    # ← Değişti
```

### Adım 2: Nav2 Params Kontrol Et
**Dosya:** `exploration_planner/config/nav2_params.yaml`

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      map_topic: /map                    # ✅ OctoMap'ten
      map_subscribe_transient_local: true
      subscribe_to_updates: true
      always_send_full_costmap: true     # ✅ Başlangıç mesajı gönder
```

### Adım 3: Topic Harita Kontrol Et
```bash
# Terminal'de çalıştır:
ros2 topic info /planner_server/global_costmap/costmap --verbose

# Çıkış şöyle olmalı:
# Publisher count: 1  ✅
# QoS profile:
#   Reliability: BEST_EFFORT ✅
#   Durability: VOLATILE ✅
```

## 🚀 SONUÇ
1. ✅ `exploration.rviz` güncellendi (Best Effort + Volatile)
2. ✅ `nav2_params.yaml` kontrol edildi (/map topic doğru)
3. ✅ Costmap şimdi RViz'de görünmeli

## 🧪 TEST
```bash
# Build
colcon build --packages-select exploration_planner

# Launch
ros2 launch exploration_planner exploration_planner.launch.py rviz:=true

# RViz'de "Nav2 Global Costmap" layer'ının görünmesi gerekir
```

## 📝 NOTLAR
- Local costmap için ayrı bir controller node kurulması gerekebilir
- OctoMap server'ı `uav_navigation/launch/navigation.launch.py`'de tanımlanmış
- Costmap yayın frekansı 2.0 Hz (iyi balance point)
