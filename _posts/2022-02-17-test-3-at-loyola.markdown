---
layout: post
title:  "fourth test with ASV in Loyola's Lake"
date:   2022-02-17 13:18:36 +0100
categories: jekyll update
---
After fixing the GPS issues at home, we parted to make a second test with the ASV and the code designed in ROS2

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/e07321b5255257a44fb60bf2b4dc7d0f75155345).



1. Build
    - The test started installing the SmartWater on the ASV, as the structure was moved from another ASV it didn’t took too long.
    - Once built and fixed some [software errors](../../../../../miscelaneous/2022-02-17-code) that were found in first place. At 10:24 we were in the wáter.

2. Errors
    - we were unable to connect to the server.

3. Fixes 
    - Fix the trim of the motors (afterwards this caused a loss of configuration of the RC controller, needed reconfiguration)
    - Frame type of the Rover (some hyperparameters needed new values)
    - Failsafes not defined

4. Achievements
    - Disregarding that we were unable to connect to the server, the code was "hard tried" using several linux terminals.
        - The drone was able to reach a point and take a sample there (first try was an error, hand input interchanged lat-lonh), second and third try were successful.
    - We were able to read the temperature of the water 15.4 degrees

5. Not so good working features
    - Finally being able to take a look at the behaviour of the drone, few things will be changed for next test:
        - RC should override program, this is due to an hyperparameter that changed after reconfiguration that was not changed again during the test.
        - New modes will be included for security measures (this will be delayed delayed until finishing a complete test)
    - The sensor module will check the string communicated via serial, and apply the correct parse
    - The USB port should be defined by reading the serial of the device connected, always connect Smartwater on USB0 and camera on USB1, for correct behaviour

{% highlight ruby %}
TODO:
- Fix parse of the sensor reading
- Fix RC override
{% endhighlight %}



{% highlight ruby %}
The sensor read was:
<=>#5C3F1CE819623CBF#SW3#7#BAT:98#WT:15.42#PH:-5.46#DO:8.0#COND:0.6#ORP:0.545#
{% endhighlight %}


No camera test was performed

The next test must be performed using all the features of the drone, the one tested today + the server, both separately has been tried successfully, but never together