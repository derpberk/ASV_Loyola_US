#!/bin/bash

#export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
#export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

#source /usr/share/colcon_cd/function/colcon_cd.sh
python3 ~/ASV_Loyola_US/src/asv_loyola_us/asv_loyola_us/startup.py
#ssh -i ~/aceti-drones-aws-key.pem -fN -L 1883:127.0.0.1:1883 ubuntu@18.169.16.66
#source /opt/ros/foxy/install/setup.bash
#source ~/ASV_Loyola_US/install/setup.sh
#source home/xavier/.bashrc
#ros2 launch asv_loyola_us system.launch.py
#ros2 launch simulator dummy_system.launch.py
