#!/bin/bash

# Script to verify ultrasonic costmap layer plugin

echo "=== Ultrasonic Costmap Layer Plugin Verification ==="
echo ""

# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. Checking if plugin library exists..."
PLUGIN_LIB="install/ultrasonic_costmap_layer/lib/libultrasonic_costmap_layer_core.so"
if [ -f "$PLUGIN_LIB" ]; then
    echo "   ✓ Plugin library found: $PLUGIN_LIB"
    ls -lh "$PLUGIN_LIB"
else
    echo "   ✗ Plugin library NOT found!"
    exit 1
fi

echo ""
echo "2. Checking plugin description file..."
PLUGIN_XML="install/ultrasonic_costmap_layer/share/ultrasonic_costmap_layer/ultrasonic_layer.xml"
if [ -f "$PLUGIN_XML" ]; then
    echo "   ✓ Plugin XML found: $PLUGIN_XML"
    cat "$PLUGIN_XML"
else
    echo "   ✗ Plugin XML NOT found!"
    exit 1
fi

echo ""
echo "3. Checking plugin symbols..."
nm install/ultrasonic_costmap_layer/lib/libultrasonic_costmap_layer_core.so | grep -i "ultrasoniclayer" | head -5

echo ""
echo "4. Checking config files..."
if [ -f "install/ultrasonic_costmap_layer/share/ultrasonic_costmap_layer/config/ultrasonic_layer_params.yaml" ]; then
    echo "   ✓ Config file found"
else
    echo "   ✗ Config file NOT found"
fi

echo ""
echo "5. Checking test launch file..."
if [ -f "install/ultrasonic_costmap_layer/share/ultrasonic_costmap_layer/test/test_ultrasonic_costmap.launch.py" ]; then
    echo "   ✓ Test launch file found"
else
    echo "   ✗ Test launch file NOT found"
fi

echo ""
echo "=== Verification Complete ==="
echo ""
echo "To test the plugin, run:"
echo "  ros2 launch ultrasonic_costmap_layer test_ultrasonic_costmap.launch.py"
