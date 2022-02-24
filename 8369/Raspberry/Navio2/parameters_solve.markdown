---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

Make sure you have followed the steps in [Raspberry pi](../Raspi.html)

Use ssh to check if the raspberry pi is connected to internet.

Afterwards check if ardurover/ardupilot is running in (systemctl)
if not, you can start it by using "sudo systemctl start ardurover"


Note: after using mission planner you may notice that if you force the close of the connection it starts a timeout and doesnt connect.
This happens cause mission planner keeps the port busy and doesnt let you connect. 2 fast solutions:

1. Softreset: Use "systemctl restart ardurover"
2. Hardreset: Restart the drone