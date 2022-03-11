---
layout: post
title:  "sixth test with ASV in Loyola's Lake"
date:   2022-03-09 13:31:41 +0100
categories: jekyll update
---

After finding a new server, we parted to make the 6th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/243871696a246f6097fd5e7b18b1e21e2e09e532).



1. Build
    - The Xavier is now in Drones 1 and 3

2. Fixes 
    - Hyperparameters configuration
    - Recalibration of Drones

3. Errors
    - Drone calibration was the main error, while we made tests mission failed due to EKF failsafe,
    - Wifi module keeps giving problems, slow connection, message loss and even connection loss at one point

4. Achievements
    - Drone 2 after calibration was able to perform the complete mission
    - Drone 1 had problems in recalibration


{% highlight ruby %}
TODO:
- 4 Drones must come out next test
- Fix problems during this mission assuring calibration before tests
- Start working on Drone 4 with the incoming Xavier NX
{% endhighlight %}

No camera test was performed

The logs can be found [here](../../../../../miscelaneous/logs 09.03.2022.rar)