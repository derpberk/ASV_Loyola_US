---
layout: post
title:  "seventh test with ASV in Loyola's Lake"
date:   2022-03-31 13:31:41 +0100
categories: jekyll update
---

After several code modifications, we parted to make the 7th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/77651fb4b55e3a2f6bedfa90347e89d63804bae8).



1. Build
    - The Xavier is now in Drones 1 to 3
    - Navio was fixed in every vehicle using a 3d printed base
    - Sensors were calibrated for every drone, and new measures were taken into account for handling sensors
    - Some minor changes

2. Fixes 
    - Now drone retries to go to destination if guided mode is lost either by call or EKF fail with several logs
    - Recalibration of Drones

3. Errors
    - None

4. Achievements
    - Drone 3 preformed perfectly, new hardware was tested.
    - Remote operation of drone was carried successfully
    - Code changes (times, ekf failure subroutine, startup and shutdown, and MQTT status and routines) worked perfectly.

{% highlight ruby %}
TODO:
- 3 Drones must come out next test
- Finish planning
{% endhighlight %}

No camera test was performed

The logs can be found [here](../../../../../miscelaneous/log 31.03.2022.log)