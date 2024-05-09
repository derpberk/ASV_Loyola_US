source /opt/ros/humble/install/setup.bash
source install/setup.bash
ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
