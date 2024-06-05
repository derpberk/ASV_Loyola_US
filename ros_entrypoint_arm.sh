#!/bin/bash
set -e

# Check if the environment variable is set
if [ -n "$MQTT_ADDR" ]; then
  NEW_MQTT_ADDR="$MQTT_ADDR"
  # Update the mqtt_addr in the YAML file
  sed -i.bak "s/mqtt_addr: .*/mqtt_addr: \"$NEW_MQTT_ADDR\"/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  echo "mqtt_addr has been updated to $NEW_MQTT_ADDR in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  # Build and source the ROS workspace
  colcon build

fi


COMMAND="$@"

source /opt/ros/humble/setup.bash
source install/setup.bash
# Launch the ROS nodes
ros2 launch asv_loyola_us system.launch.py & ros2 launch mavros apm.launch fcu_url:=tcp://192.168.1.203:5678 gcs_url:=udp://@

# Execute the main process
exec $COMMAND
