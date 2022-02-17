---
layout: post
title:  "second test with ASV in Loyola's Lake"
date:   2022-02-10 15:24:12 +0100
categories: jekyll update
---

After fixing the MQTT communication issues at home, we parted to make a second test with the ASV and the code designed in ROS2

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/53bd44e43307ef7dc0a3df032ed6c03f6623a766).

1. Build
- The structure was reatached, the Lora antenna that was missing was introduced in the drone box.
- The position of the Navio2 was changed, as having it facing downwards gave us EKF problems


3. Errors
- We los almost an hour debugging GPS position as there was no lecture, finding the solution at the end.
- The Lora antenna was missing.
- The GPS and positioning data read from the drone was 0 0 0. The EKF retransmits error.

{% highlight ruby %}
TODO:
- Battery read needs to be fixed
- GPS reading needs to be fixed
{% endhighlight %}

The camera test has proceeded using the RC controller, the ROS2 code will be given another chance next week.
It has been decided that all the tools needed to check the drone and GPS will be carried home for debug, so that we can complete next test.


The needed configuration that will be carried for next nmew drones is going to be published in the [configuration guide](../../../../../Raspberry/Raspi.html)