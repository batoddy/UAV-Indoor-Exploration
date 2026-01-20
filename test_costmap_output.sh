#!/bin/bash

echo "🔍 Costmap verisini kontrol ediyorum..."
sleep 2

echo -e "\n📊 /map topic'i kontrol et:"
ros2 topic echo /map --once 2>/dev/null | head -30 || echo "❌ /map topic verisi yok"

echo -e "\n📊 Costmap topic'i kontrol et:"
ros2 topic echo /planner_server/global_costmap/costmap --once 2>/dev/null | head -50 || echo "❌ Costmap topic verisi yok"

echo -e "\n✅ Test tamamlandı"
