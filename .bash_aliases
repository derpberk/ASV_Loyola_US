alias alone='kill $(pgrep bash)'
alias falone='kill -9 $(pgrep bash)'

alias cbuild='colcon build --packages-select '  

alias roscd='cd ~/ASV_Loyola_US'
alias s='source ~/ASV_Loyola_US/install/setup.bash'

alias hearall='ros2 topic echo -f '
alias status='ros2 topic echo /ASV/status'

alias mqttssh='ssh -i ~/AzureDronesKey.pem -fN -L 1883:127.0.0.1:1883 azuredrones@dronesloyolaus.eastus.cloudapp.azure.com'
alias nodered='ssh -i ~/AzureDronesKey.pem -fN -L 1880:127.0.0.1:1880 azuredrones@dronesloyolaus.eastus.cloudapp.azure.com'

alias check_ssh_running='ps -aux | grep "ssh -i"'

buall()
{
roscd;
colcon build --symlink-install;
}

sbuall()
{
buall;
s;
}
sudo -S <<< "xavier" chmod 666 /dev/ttyUSB0; #this is not an alias, it will be executed in bashrc, for Smart Water enable

function roslaunch(){
ros2 launch asv_loyola_us system.launch.py;
}

#function roslaunch(){
#ros2 launch simulator dummy_system.launch.py;
#}

function restart_asv(){
    sudo -S <<< "xavier" systemctl restart asv_start.service; 
}

function update_time(){
    sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z";
}

function update_asv(){
    roscd;
    mkdir temp;
    cp ./src/asv_loyola_us/config/config.yaml ./temp/config.yaml;
    git pull -f;
    cp ./temp/config.yaml ./src/asv_loyola_us/config/config.yaml;
    sbuall;
}