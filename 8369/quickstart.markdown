---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
#title: Quickstart
---

1. Install [ROS2](https://docs.ros.org/en/foxy/Installation.html) (recomended Ubuntu 20.4 [$] binaries, but also works on windows[%]) and configure it in bash
2. Install [colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
3. Clone this git in any folder (to avoid path errors use ${HOME})

   `git clone https://github.com/AloePacci/ASV_Loyola_US`
4. Change to repository folder

   `cd ./ASV_Loyola_US`
5. Resolve package dependencies 

   `rosdep install -i --from-path src --rosdistro foxy -y`
6. Build the package

   `colcon build --symlink-install`


- (optional)
  - add the [aliases](./aliases.html) to your `~/.bashrc` for commodity
  - indicate a specific `ROS_DOMAIN_ID` if you want to work with multiple drones separately in the same network