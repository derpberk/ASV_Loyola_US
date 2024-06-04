#!/bin/bash
set -e

# Check if enough arguments are provided
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <new_mqtt_addr> <command>"
  exit 1
fi

NEW_MQTT_ADDR="$1"
shift
COMMAND="$@"

# Update the mqtt_addr in the YAML file
sed -i.bak "s/mqtt_addr: .*/mqtt_addr: \"$NEW_MQTT_ADDR\"/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"

echo "mqtt_addr has been updated to $NEW_MQTT_ADDR in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"

colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch asv_loyola_us system.launch.py & ros2 launch mavros apm.launch fcu_url:=tcp://192.168.1.203:5678 gcs_url:=udp://@

# Execute the main process
exec $COMMAND