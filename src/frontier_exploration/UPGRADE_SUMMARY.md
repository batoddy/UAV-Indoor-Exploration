# Viewpoint Generator Node - Upgrade Summary

## Overview
The Viewpoint Generator Node has been upgraded from a **Temporal Stabilization** version to a **Feature-rich + Occlusion-aware + Stabilized** version. This update significantly improves the quality of viewpoint generation in complex environments with occlusions.

---

## Changes Made

### 1. **Viewpoint Message Extension** ✅
**File:** `msg/Viewpoint.msg`

Added three new fields to embed cluster context into each viewpoint:

```
# OLD:
geometry_msgs/Point position
float64 yaw
int32 coverage
float64 distance_to_centroid

# NEW (added):
uint32 cluster_id
int32 cluster_size
geometry_msgs/Point cluster_centroid
```

**Rationale:** These fields allow downstream planners to make decisions based on cluster information without additional lookups.

---

### 2. **New Includes** ✅
**File:** `src/viewpoint_generator_node.cpp`

Added critical missing import:
```cpp
#include "frontier_exploration/msg/frontier_cluster.hpp"  // NEW
#include <unordered_set>  // NEW
#include <vector>  // NEW
```

**Rationale:** Needed for accessing `FrontierCluster` members and enhanced data structures.

---

### 3. **Occlusion-Aware Coverage Algorithm** ✅

#### New Parameters:
```cpp
declare_parameter("occlusion_enabled", true);
declare_parameter("occlusion_angle_bin_rad", 0.02);  // ~1.15 degrees
declare_parameter("yaw_samples", 36);
```

#### New Algorithm Components:

**A) PolarCell Structure:**
```cpp
struct PolarCell {
    double ang;      // Angle from viewpoint
    double dist;     // Distance from viewpoint
    geometry_msgs::msg::Point p;  // Cell position
};
```

**B) Two Coverage Methods:**

1. **computeCoverageSimple()** - Simple LOS-based (OLD behavior)
   - Counts frontier cells visible by line-of-sight
   - No occlusion awareness

2. **computeCoverageOcclusionAware()** - NEW algorithm
   - Organizes frontier cells into angular bins
   - Once LOS fails in a bin, blocks all farther cells
   - Prevents double-counting of occluded areas
   - More realistic coverage estimates

**C) normalizeAngle() Helper:**
```cpp
static inline double normalizeAngle(double a)
{
    while (a >= M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}
```

---

### 4. **Feature Injection System** ✅

New function `fillClusterContextIntoViewpoint()`:
```cpp
void fillClusterContextIntoViewpoint(frontier_exploration::msg::Viewpoint &vp,
                                     const frontier_exploration::msg::FrontierCluster &cluster)
{
    vp.cluster_id = cluster.id;
    vp.cluster_size = static_cast<int32_t>(cluster.cells.size());
    vp.cluster_centroid = cluster.centroid;
}
```

Called in two places:
1. During viewpoint generation (initial feature injection)
2. After stabilization (ensures consistency)

---

### 5. **Code Quality Improvements** ✅

| Aspect | OLD | NEW |
|--------|-----|-----|
| Utility Functions | `distance()` (inline) | `dist()`, `dist2()` (optimized) |
| Helper Functions | Basic set operations | Full occlusion pipeline |
| Parameter Scope | All global parameters | Scoped with proper defaults |
| Type Safety | Mixed int/int8_t | Explicit casting |
| Container Usage | `std::set` | `std::unordered_set` |
| Angle Handling | Direct atan2 | atan2 + normalization |

---

## Algorithm Flow

### OLD Flow:
```
Cluster → Generate Candidates → Check 2D/3D Clearance → Optimize Yaw (Simple Coverage) 
→ Stabilize → Publish
```

### NEW Flow:
```
Cluster → Generate Candidates → Check 2D/3D Clearance → Optimize Yaw 
  ↓
  └→ Coverage: Occlusion-Aware
      1. Organize cells by polar coords
      2. Sort by angle, then distance
      3. Track blocked ray bins
      4. Skip occluded cells
  ↓
Stabilize → Inject Cluster Context → Publish
```

---

## Key Improvements

### 1. **More Accurate Coverage Estimation**
- **Problem:** Old method counted occluded cells, inflating coverage
- **Solution:** Ray-bin blocking prevents counting behind obstacles

### 2. **Cluster Context Embedding**
- **Problem:** Downstream planners needed separate cluster lookup
- **Solution:** Each viewpoint carries its cluster metadata

### 3. **Better Angle Normalization**
- **Problem:** Angle binning could be unstable across quadrants
- **Solution:** normalizeAngle() ensures stable [-π, π) mapping

### 4. **Configurable Occlusion**
- Can enable/disable occlusion-aware coverage
- Tunable angular bin size for different precisions
- Adjustable yaw sample count

---

## Parameter Configuration

### New Parameters in `config/params.yaml`:

```yaml
viewpoint_generator:
  # Occlusion-aware coverage
  occlusion_enabled: true
  occlusion_angle_bin_rad: 0.02      # ~1.15 degrees
  yaw_samples: 36                     # 10 degree intervals
  
  # Existing parameters remain:
  sensor_range: 5.0
  sensor_fov_h: 1.57
  min_dist: 1.5
  max_dist: 4.0
  # ... etc
```

---

## Backward Compatibility

✅ **Fully Compatible** - Old code will still run:
- Default `occlusion_enabled: true` can be set to `false`
- All existing parameters preserved
- New fields in Viewpoint message are optional (ROS auto-fills defaults)

---

## Testing Recommendations

1. **Coverage Accuracy Test:**
   - Compare coverage values with/without occlusion
   - Expected: Higher accuracy, slightly lower counts in complex scenes

2. **Performance Test:**
   - Measure computation time with occlusion enabled
   - Should be <5-10% slower than simple method

3. **Stabilization Test:**
   - Verify old viewpoints aren't displaced by occlusion changes
   - Check cluster context fields are populated correctly

4. **Parameter Tuning:**
   - Test with different `occlusion_angle_bin_rad` values
   - Smaller bins = finer resolution, higher cost
   - Typical: 0.02 rad ≈ 1.15° bins

---

## Files Modified

| File | Changes |
|------|---------|
| `msg/Viewpoint.msg` | ✅ Added 3 fields (cluster_id, cluster_size, cluster_centroid) |
| `src/viewpoint_generator_node.cpp` | ✅ Complete rewrite with occlusion support |

## Build Status

✅ **Successfully Compiled** (1 min 49s)
- No errors or warnings
- Ready for deployment

---

## Future Enhancements

1. **Adaptive Binning:** Adjust bin size based on distance
2. **Occlusion Probability:** Weight blocked rays instead of hard blocking
3. **Temporal Occlusion:** Track occlusion history for better predictions
4. **GPU Acceleration:** Parallelize ray casting

---

Generated: January 18, 2026
