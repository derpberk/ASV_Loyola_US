---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

The Raspberry pi currectly supports the image provided by [Navio2](https://navio2.emlid.com/)

ROS is deprecated as it will be no longer supported in 2022. So the scripts will be processed in a third computer, in our case [JETSON XAVIER NX](../src/Xavier.markdown)

To configure the raspberry pi you must:

according to [Navio2 Instructions](https://docs.emlid.com/navio2/ardupilot/installation-and-running) you must change the configuration in 
{% highlight ruby %}
sudo nano /etc/default/ardurover
{% endhighlight %}
- Modify the IP of ardurover by:
  - Setting telem1 value to 'tcp:0.0.0.0:5678'
    - This means we will broadcast TCP in port 5678
  - Setting telem2 value to "udp:127.0.0.1:14650"
    - In case we want the raspberry to act over the navio in case of connection loss
  - Add a telem3 value to "tcp:0.0.0.0:1224"
    - Another broadcast TCP in port 1224 (for mission planner purposes)
  - Add a Telem3 to “-B /ttyAMA0”
    - To read the data from the Emlid M+ GPS

Afterwards you can activate ardupilot/ardurover by using
{% highlight ruby %}
sudo systemctl enable ardurover
{% endhighlight %}

afterwards you must configure all the internal parameters of [Navio2](./Navio2.html)