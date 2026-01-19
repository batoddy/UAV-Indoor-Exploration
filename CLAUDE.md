# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS2 UAV Autonomous Exploration System** that performs frontier-based autonomous exploration using a PX4-based drone with a depth camera for indoor/enclosed space exploration.

### Simulation Environment
- **Platform**: PX4 SITL with x500 quadrotor in Gazebo
- **Sensor**: Depth camera mounted on UAV
- **Control**: cmd_vel → velocity setpoint conversion to PX4

### System Flow
1. **Frontier Detection** (frontier_exploration): Detect frontiers in occupancy grid
2. **Viewpoint Generation** (frontier_exploration): Generate observation viewpoints with utility scores
3. **Greedy Selection** (exploration_planner): Select best viewpoint using cost function
4. **Nav2 Path Planning** (exploration_planner): Plan collision-free path using Nav2 global/local planners
5. **Yaw Injection** (exploration_planner): Inject target yaw into path (starts rotating 1.5m before goal)
6. **Trajectory Following** (exploration_planner): Convert path to cmd_vel commands
7. **PX4 Bridge** (cmd_vel_to_px4): Convert cmd_vel to PX4 velocity setpoints

### Nav2 Integration
The system uses Nav2 for path planning with:
- **Global Costmap**: Built from OccupancyGrid (2D projection of OctoMap)
- **Local Costmap**: Depth camera slice at flight height (~1.5m)
- **Planners**: Global planner (A*, NavFn, etc.) + Local planner (DWB or alternatives)
- **Yaw Injection**: Path post-processing to blend yaw orientation before reaching goal point

## Build Commands

```bash
# Build entire workspace
cd /home/batoddy/uav_ws
colcon build

# Build specific package
colcon build --packages-select frontier_exploration
colcon build --packages-select exploration_planner

# Build with dependencies
colcon build --packages-up-to exploration_planner

# Source workspace after building
source install/setup.bash
```

## Running the System

The system requires multiple launch files to be run in sequence:

```bash
# Terminal 1: Start OctoMap server (converts PointCloud to 2D map)
ros2 launch uav_navigation navigation.launch.py use_sim_time:=true

# Terminal 2: Start frontier detection pipeline
ros2 launch frontier_exploration frontier_exploration.launch.py use_sim_time:=true

# Terminal 3: Start exploration planner (path planning + trajectory following)
ros2 launch exploration_planner exploration_planner.launch.py use_sim_time:=true rviz:=true

# Terminal 4: Start PX4 bridge (cmd_vel → PX4 commands)
ros2 launch cmd_vel_to_px4 px4_bridge.launch.py
```

Control exploration via services:
```bash
ros2 service call /exploration/start std_srvs/srv/Trigger
ros2 service call /exploration/stop std_srvs/srv/Trigger
```

## Architecture

### Data Flow Pipeline

```
Depth Camera → PointCloud → OctoMap → 2D OccupancyGrid (/map)
                                            ↓
                              ┌─────────────┴─────────────┐
                              ↓                           ↓
                    Frontier Detector            Nav2 Global Costmap
                              ↓                           ↓
                    Viewpoint Generator          Nav2 Local Costmap (depth slice)
                              ↓                           ↓
                    Greedy Selector ──→ Goal Pose ──→ Nav2 Planners
                                                          ↓
                                                   Planned Path
                                                          ↓
                                                   Yaw Injector
                                                          ↓
                                               Trajectory Follower
                                                          ↓
                                                      cmd_vel
                                                          ↓
                                              PX4 Bridge (velocity setpoint)
                                                          ↓
                                                   PX4 SITL → Gazebo
```

### Package Responsibilities

| Package | Purpose |
|---------|---------|
| **frontier_exploration** | Detects frontiers, generates viewpoints with occlusion-aware coverage |
| **exploration_planner** | Greedy frontier selection, Nav2 path planning, yaw injection, trajectory following |
| **uav_navigation** | OctoMap server configuration, 2D map projection from 3D voxel map |
| **cmd_vel_to_px4** | Bridges geometry_msgs/Twist to PX4 velocity setpoints |
| **px4_msgs** | PX4 message definitions (submodule) |
| **px4-offboard** | PX4 offboard control examples |
| **sim_topic_bridge** | Gazebo-ROS2 topic bridging |

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/points` | PointCloud2 | Depth camera point cloud input |
| `/map` | OccupancyGrid | 2D projected map from OctoMap |
| `/octomap_binary` | Octomap | 3D voxel map for collision checking |
| `/frontier_clusters` | FrontierArray | Detected frontier clusters |
| `/frontier_clusters_with_viewpoints` | FrontierArray | Clusters with computed viewpoints |
| `/goal_pose` | PoseStamped | Target viewpoint for Nav2 |
| `/exploration/planned_path` | nav_msgs/Path | Collision-free path from Nav2 |
| `/exploration/path_with_yaw` | nav_msgs/Path | Path with injected yaw orientations |
| `/cmd_vel` | Twist | Velocity commands for drone |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | PX4 velocity setpoint |
| `/odom` | Odometry | UAV state feedback |

## Configuration Files

- **frontier_exploration**: `src/frontier_exploration/config/params.yaml`
  - Frontier clustering (min/max sizes, thresholds)
  - Viewpoint sampling (distances, angles, sensor FOV)
  - Robot footprint for collision checking

- **exploration_planner**: `src/exploration_planner/config/params.yaml`
  - Motion limits (v_max, yaw_rate_max, accelerations)
  - Frontier selection weights (w_distance, w_size, w_angle, w_coverage)
  - Trajectory follower gains (kp_linear, kp_yaw)
  - Height control (target_height: 1.5m)

- **Nav2 planner**: `src/exploration_planner/config/nav2_params.yaml`
  - Global costmap configuration
  - Path planner plugin settings

## Custom Messages

Located in `src/frontier_exploration/msg/` and `src/exploration_planner/msg/`:

- **Viewpoint.msg**: Observation position with yaw, coverage, and embedded cluster context (cluster_id, cluster_size, cluster_centroid)
- **FrontierCluster.msg**: Cluster with cells, centroid, PCA components, viewpoints
- **FrontierArray.msg**: Array of frontier clusters
- **ExplorationStatus.msg**: Tour plan with waypoints, progress tracking
- **Trajectory.msg**: Poses + velocities + timestamps for smooth following

## Key Algorithms

1. **Frontier Detection** (`frontier_detector_node`): BFS clustering with PCA-based splitting for large clusters

2. **Viewpoint Generation** (`viewpoint_generator_node`):
   - Cylindrical sampling around cluster centroids
   - 2D footprint collision check on OccupancyGrid
   - 3D collision check on OctoMap (optional)
   - Occlusion-aware coverage computation (configurable via `occlusion_enabled` param)

3. **Frontier Selection** (`global_tour_planner_node`): Greedy weighted cost:
   ```
   cost = w_distance * dist + w_size * (1/size) + w_angle * angle + w_coverage * (1/coverage)
   ```

4. **Nav2 Path Planning**: Global planner for obstacle-free path, local planner (DWB) for reactive control

5. **Yaw Injection** (`path_yaw_injector_node`): Post-processes Nav2 path to blend target yaw orientation starting ~1.5m before goal. This allows the UAV to face the frontier upon arrival without stopping to rotate.

6. **Trajectory Following** (`trajectory_follower_node`): PID control at 30Hz with velocity smoothing and height hold at target altitude

## Dependencies

- ROS2 (Humble or later)
- Nav2 stack (planner_server, controller_server, costmap_2d, lifecycle_manager, DWB controller)
- OctoMap / octomap_server
- PX4 Autopilot SITL (via px4_msgs)
- Gazebo Sim (for simulation)
- TF2 for coordinate transforms
