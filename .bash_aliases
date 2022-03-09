alias pycharm='nohup pycharm.sh &'

start_vehicle()
{
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool  "value: true">
}

stop_vehicle()
{
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool  "value: false>
}


alias cbuild='colcon build --packages-select '  

alias roscd='cd ~/ASV_Loyola_US'

