#!/bin/bash
set -e

# Check if the MQTT_ADDR environment variable is set
if [ -n "$MQTT_ADDR" ]; then
  NEW_MQTT_ADDR="$MQTT_ADDR"
  # Update the mqtt_addr in the YAML file
  sed -i.bak "s/mqtt_addr: .*/mqtt_addr: \"$NEW_MQTT_ADDR\"/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  echo "mqtt_addr has been updated to $NEW_MQTT_ADDR in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  BUILD_NEEDED=true
fi

# Check if the DEBUG environment variable is set
if [ -n "$DEBUG" ]; then
  # Convert the DEBUG value to lowercase to handle true/false correctly
  DEBUG_LOWER=$(echo "$DEBUG" | tr '[:upper:]' '[:lower:]')
  if [ "$DEBUG_LOWER" = "true" ] || [ "$DEBUG_LOWER" = "false" ]; then
    # Update the debug in the YAML file with boolean value
    sed -i.bak "s/debug: .*/debug: $DEBUG_LOWER/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
    echo "debug has been updated to $DEBUG_LOWER in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
    BUILD_NEEDED=true
  else
    echo "Invalid value for DEBUG: $DEBUG. Expected 'true' or 'false'."
    exit 1
  fi
fi

# Build and source the ROS workspace if needed
if [ "$BUILD_NEEDED" = true ]; then
  colcon build
fi


COMMAND="$@"

source /opt/ros/humble/setup.bash
source install/setup.bash
# Launch the ROS nodes
ros2 launch asv_loyola_us system.launch.py & ros2 launch mavros apm.launch fcu_url:=tcp://192.168.1.104:5788 gcs_url:=udp://@

# Execute the main process
exec $COMMAND
