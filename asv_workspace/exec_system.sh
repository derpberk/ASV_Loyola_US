source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch asv_loyola_us system.launch.py & ros2 launch mavros apm.launch fcu_url:=tcp://192.168.1.203:5688 gcs_url:=udp://@