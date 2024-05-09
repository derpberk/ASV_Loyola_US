source /opt/ros/humble/install/setup.bash
source install/setup.bash
ros2 service call /zed/zed_node/start_svo_rec zed_interfaces/srv/StartSvoRec "{svo_filename: '/root/recorded_svo/test_Alamillo.svo', compression_mode: '2', target_framerate: '30', bitrate: '60000'}"
