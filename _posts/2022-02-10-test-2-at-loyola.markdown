---
layout: post
title:  "second test with ASV in Loyola's Lake"
date:   2022-02-03 10:00:00 +0100
categories: jekyll update
---

After fixing the MQTT communication issues at home, we parted to make a second test with the ASV and the code designed in ROS2

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/53bd44e43307ef7dc0a3df032ed6c03f6623a766).

1. Build
- The structure was reatached, the Lora antenna that was missing was introduced in the drone box

2. Doubts:
- The code works, the modes and actions needed, work perfectly at first sight, everything according to simmulations.
- After driving the drone to the water with the intention to test the whole functioning we found an incompatibility.
  - The connection with the server was out, mainly because:
    - The code was adapted to simulation, the MQTT wasn't called (issue fixed online)
    - The paho-mqtt library presented incompatibilities with some functions (mainly, "client.is_connected()")
    - The fix of the error + not having battery value, caused the status topic not to pubblish as it had void data, so it was changed to pubblish dummy data if empty, to know the status of the ASV,

3. Errors
- The ASV wasn't able to connect to the server. There was no publication.

{% highlight ruby %}
TODO:
- Battery read needs to be fixed
- Connection with the server needs to be fixed
{% endhighlight %}

It has been decided to repeat the test next week in order to fix the actual errors and complete the test.