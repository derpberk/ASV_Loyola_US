---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
Once the drone starts a [python script](https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/startup.py) runs, checking internet connection and sending a waiting for server message to the server.

Once a message is received from the server, it will execute the [ros2 launch](./launch/launch.html) in a terminal environment.

when the launch finishes (either from shutdown or system crash) there exists a procedure (defined in [ros2 launch](./launch/launch.html)) to restart automatically the startup python script.

To achieve this, you must, first put the [asv_start.service](https://github.com/AloePacci/ASV_Loyola_US/blob/main/other%20code%20for%20development/asv_startup.service) file in /etc/systemd/system/

$ sudo cp ~/ASV_Loyola_US/other\ code\ for\ development/asv_start.service  /etc/systemd/system/
$ sudo systemctl daemon-reload
$ sudo systemctl enable asv_start.service
$ sudo cp ~/ASV_Loyola_US/other\ code\ for\ development/asv_start.sh  /usr/bin/

