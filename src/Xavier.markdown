---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
The code running in the Jetson Xavier NX is the one that you can find in this [github repository](https://github.com/AloePacci/ASV_Loyola_US)

The whole code has been made in Python with a ROS2 foxy architecture.

- For more info about [ROS2 foxy](https://docs.ros.org/en/foxy/index.html)
- For more info about what the code does check [Behaviour](../funcionamiento.html)

To accomplish the requirements we have created 3 packages.
- [ASV_interfaces](./asv_interfaces/asv_interfaces.html) holds all the interfaces for topics, services and actions that our robot uses
- [ASV_Loyola_US](./asv_loyola_us/asv_loyola_us.html) holds the code that governs the ASV behaviour
- [Simulator](./simulator/visualizer.html) emulates some sensors for debug procedures

The system runs a [service at startup](./startup.html) to be able to initialize the drone remotely.


- Introduction to python
- Introduction to ardupilot
- Introduction to dronekit
- Introduction to Ros2
- Ros2 second steps
- map creation