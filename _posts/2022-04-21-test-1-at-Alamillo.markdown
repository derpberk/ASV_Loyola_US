---
layout: post
title:  "first test with ASV in Alamillo's Lake"
date:   2022-04-21 11:24:16 +0100
categories: jekyll update
---

After several code modifications, we parted to make the 8th test at Loyola's Lake

the code used can be found in this [commit](https://github.com/AloePacci/ASV_Loyola_US/tree/9c5a59dcd38e5e84f7fda99f93e5ec41ac5b9e26).



1. Build
    - Alamillo's map was introduced
    - Drone now resumes and takes ekf more into account executing new subroutines
    - There is more info about the drone status

2. Fixes 
    - Sensor node avoided lectures due to len==0, need fix

3. Errors
    - Drone 3 was unable to perform properly, it had several GPS issues that made it unable to reach destinations safely as well that EKF fails
    - start was a bit slow due to a sensor node issue

4. Achievements
    - Drones 1 and 2 did missions and took samples without any issue.

{% highlight ruby %}
TODO:
- Drone 3 must be fixed
- More friendly way to update parameters, find a way to make permanent parameters
- Fix map load
- Fix empty sensor read
- Add point calculation at start of executing phase in 
- It is advised to buy new GPS antennas and use a metalic board on the drones.  
{% endhighlight %}

few camera tests were performed

Notas:
- Conectar el navio por ethernet para evitar perdidas de conexion por router
- Timeouts de toma de sensores y de conexion a internet, devolviendo modo manual
- Estudiar la configuración del navio, encontrar la forma de arrebatar el control al código (se puede utilizar dronekit para ello si ardupilot no lo permite (ardupilot es mejor opcion))
    

The logs can be found [here](../../../../../miscelaneous/log2404.rar)

![video](../../../../../miscelaneous/drona.gif)