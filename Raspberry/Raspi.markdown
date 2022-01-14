---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

The Raspberry pi currectly supports the image provided by [Navio2](https://navio2.emlid.com/)

ROS is deprecated as it will be no longer supported in 2022

To configure the raspberry pi you must:

- Modify the IP of ardurover by:
  - Setting telem1 value to 'tcp:0.0.0.0:5678'
    - This means we will broadcast TCP in port 5678
  - Setting telem2 value to "udp:127.0.0.1:14650"
    - In case we want the raspberry to act over the navio in case of connection loss
- Remember to configure navio and GPS