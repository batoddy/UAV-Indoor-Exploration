#!/bin/bash

echo "================================================"
echo "🔍 COSTMAP DIAGNOSTICS - FULL CHECK"
echo "================================================"

echo -e "\n1️⃣  CHECK IF TOPICS EXIST:"
echo "Available map-related topics:"
ros2 topic list | grep -E "map|costmap" || echo "❌ No map/costmap topics found!"

echo -e "\n2️⃣  /map TOPIC STATUS (OctoMap → 2D Map):"
if ros2 topic list | grep -q "^/map$"; then
    echo "✅ /map topic exists"
    echo "QoS Info:"
    ros2 topic info /map --verbose 2>/dev/null | grep -E "Publisher|Reliability|Durability"
    
    echo -e "\nSample data from /map:"
    timeout 1 ros2 topic echo /map --once 2>/dev/null | head -15 || echo "❌ No data from /map!"
else
    echo "❌ /map topic NOT found - OctoMap server may not be running!"
fi

echo -e "\n3️⃣  COSTMAP TOPIC STATUS (Nav2 Publisher):"
if ros2 topic list | grep -q "costmap/costmap"; then
    echo "✅ Costmap topic exists"
    echo "QoS Info:"
    ros2 topic info /planner_server/global_costmap/costmap --verbose 2>/dev/null | grep -E "Publisher|Reliability|Durability"
    
    echo -e "\nSample data from costmap:"
    timeout 1 ros2 topic echo /planner_server/global_costmap/costmap --once 2>/dev/null | head -20 || echo "❌ No data from costmap!"
else
    echo "❌ Costmap topic NOT found - planner_server may not be running!"
fi

echo -e "\n4️⃣  NODE STATUS:"
echo "Nodes running:"
ros2 node list | grep -E "planner|lifecycle|octomap" || echo "❌ Required nodes not running!"

echo -e "\n5️⃣  CHECKING FOR ERRORS:"
echo "Looking for warnings/errors in planner_server..."
# This requires the node to be still running, check logs
if [ -d "$HOME/.ros/log" ]; then
    LATEST_LOG=$(ls -t $HOME/.ros/log/*/planner_server* 2>/dev/null | head -1)
    if [ ! -z "$LATEST_LOG" ]; then
        echo "Latest errors/warnings:"
        grep -i "error\|warn" "$LATEST_LOG" 2>/dev/null | tail -5 || echo "No errors found in logs"
    fi
fi

echo -e "\n================================================"
echo "✅ Diagnostics complete"
echo "================================================"
