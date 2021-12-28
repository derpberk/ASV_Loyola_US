To use this packet you need to have installed Ardupilot SITL https://ardupilot.org/dev/docs/SITL-setup-landingpage.html#sitl-setup-landingpage

git clone https://github.com/mavlink/mavlink-gbp-release.git --branch release/foxy/mavlink mavlink

ros2 run mavros mavros_node --ros-args --params-file mavros_param.yaml 

