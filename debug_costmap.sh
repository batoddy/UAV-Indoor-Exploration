#!/bin/bash
# Debug script - Nav2 Costmap QoS ve Topic durumunu kontrol et

echo "========================================"
echo "NAV2 COSTMAP DIAGNOSTICS"
echo "========================================"

echo -e "\n1️⃣  RUNNING NODES:"
ros2 node list | grep -E "planner|lifecycle"

echo -e "\n2️⃣  COSTMAP TOPICS:"
ros2 topic list | grep costmap

echo -e "\n3️⃣  TOPIC QoS INFO - /planner_server/global_costmap/costmap:"
ros2 topic info /planner_server/global_costmap/costmap --verbose

echo -e "\n4️⃣  TOPIC QoS INFO - /map (OctoMap):"
ros2 topic info /map --verbose

echo -e "\n5️⃣  TEST: Costmap message kontrol et:"
timeout 2 ros2 topic echo /planner_server/global_costmap/costmap --once 2>/dev/null | head -20

echo -e "\n6️⃣  ERROR CHECK - Planner server logs:"
ros2 node info /planner_server 2>/dev/null | grep -A 5 "error\|warn\|Error" || echo "No errors in logs"

echo -e "\n✅ Diagnostics tamamlandı"
