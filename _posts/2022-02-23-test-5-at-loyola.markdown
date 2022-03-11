---
layout: post
title:  "fourth test with ASV in Loyola's Lake"
date:   2022-03-04 13:31:41 +0100
categories: jekyll update
---

we parted to make the 5th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/4febbb00387e345b4a64024f95b3fa0cac0d161c).



1. Build
    - The Dron 2 was built with a Jetson Nano instead of a Xavier NX, some structure changes including a power PCB

2. Fixes 
    - Now we take several samples at the same point

3. Errors
    - GPS was hard to achieve in the new drone, due to a problem in hardware connections.
    - In this commit a few variables in sensor module are missing, coding error
    - Drone 2 has power issues that restart the Jetson Nano
    - Internet keeps giving problems.

4. Achievements
    - Jetson Nano is 100% functional.


{% highlight ruby %}
TODO:
- Fix changes
- Next test 3 Drones must go into the water
{% endhighlight %}

No camera test was performed