---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
For the package to work some conditions must meet
- the contents of setup.py and setup.conf information field must be the same
- package.xml must have as dependencies all the (not included in python installation) libraries and other packages
- All the nodes running must be defined in setup.py

For convenience (according to ROS2) is recommended to place all the code inside `asv_loyola_us` folder

This package is already configured to:
- Add to rospath all the `*.launch.py` files created inside the folder `launch`
- Add to rospath all the `*.yaml` files created inside the folder `config`
- make nodes able to use user libraries placed in folder `submodulos`

notes:
- if you build using `colcon build --symlink-install` you don't have to build again for the changes to take effect
- you must build the package before sourcing it

you may also want to see [package used interfaces](../../interfaces.html)
