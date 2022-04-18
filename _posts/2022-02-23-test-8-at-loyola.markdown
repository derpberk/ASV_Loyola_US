---
layout: post
title:  "eigth test with ASV in Loyola's Lake"
date:   2022-04-07 13:31:41 +0100
categories: jekyll update
---

After several code modifications, we parted to make the 8th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/commit/92a912a174d8e7405d2657add5a27d4f26cb55b9).



1. Build
    - Sensors were recalibrated
    - Planner was introduced
2. Fixes 
    - Now Planner skips point if it is unable to find a path instead of bypassing
    - Recalibration of Drones

3. Errors
    - GPS was lost several times
    - Drone 3 performed at start and stoped working midway, before next test its recommended to check behaviour
    - RC was recalibrated
    - Some cables were needed to fix on the air due to bad connection (drone 2).
    - Mission got stuck unabling the drone to continue after communication loss.

4. Achievements
    - Planner performed without issues, [see this log](../../../../../miscelaneous/planner_log_8.html)

{% highlight ruby %}
TODO:
- Code must react and stop Drone is GPS sends bad values or there is a bad health to avoid collisions
- GPS antenna will be placed in a metal base
- Emlid parameters should be changed
- fix drone resuming
{% endhighlight %}

- Notes
  - Nodered allows one user to be connected at a time, so multidrone actuation is hard, suggestion to divide drones in 3 tabs to allow access from multiple devices at the cost of more computation.

few camera tests were performed

The logs can be found [here](../../../../../miscelaneous/test8.rar)