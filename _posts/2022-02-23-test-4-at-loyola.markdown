---
layout: post
title:  "fourth test with ASV in Loyola's Lake"
date:   2022-02-23 13:31:41 +0100
categories: jekyll update
---

After finding a new server, we parted to make the 4th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/4febbb00387e345b4a64024f95b3fa0cac0d161c).



1. Build
    - The pump and tank were deprecated, a new structure to hold the sensors directly into the water was installed in the drone.
    - Due to GPS problems a second antenna was connected to the drone, directly to the Navio2

2. Fixes 
    - Fix the trim of the motors in manual control
    - Fixed battery lecture. Now we get a lecture but it is not real, just orientative

3. Errors
    - At some point during the Test the GPS failed, no satellites were in our area so the drone was unable to reach new points, however it worked perfectly until and after.

4. Achievements
    - We were finally able to try the whole system, and it was achieved successfully!
    - New integrated functionalities were also tried and really thanked for, as they became really useful 

5. Not so good working features
    - The RC is unable to override program by itself. Mode doesn't change to manual.
    - GPS loss should be managed
    - Simple-point resume should take into account the point being (0.0, 0.0) as a point not to go to


{% highlight ruby %}
TODO:
- Scale battery read
- Make pump become a configurable parameter in the config file, to avoid unnecessary wait if the pump is not installed (and take more than 1 sample at each samplepoint)
- Try the system in the jetson Nano
- Planing for static obstacles
{% endhighlight %}

No camera test was performed

The sensor reads can be found [here](../../../../../miscelaneous/2022-02-23-sensor_reads.html)