---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Mission Node</H1> 
<pre>
Services
/<a href="./services/close_asv.html">close_asv</a>
/<a href="./services/change_mission_mode.html">change_mission_mode</a>
/<a href="./services/new_samplepoint.html">new_samplepoint</a>
</pre>
<pre>
Topics
/
</pre>
<pre>
Actions
/
</pre>

This node contains the state machine of the robot

[code](https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py)


<pre>
functions
<a href="#arm_vehicle">arm_vehicle(value)</a>
<a href="#new_mission_mode">new_mission_mode(request, response)</a>
<a href="#get_next_wp">get_next_wp()</a>
</pre>


<pre>
variables
<a id="self.mission_mode">mission_mode</a>
<a id="self.samplepoints">samplepoints</a>
</pre>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ARM VEHICLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>arm_vehicle(value)</H3>
<a id="arm_vehicle"></a>
This function calls the service [arm_vehicle](./404) to arm the vehicle
- params
  - value: True to arm, false to disarm
- output
  - True when finished

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW MISSION MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>new_mission_mode(request, response)</H3>
<a id="new_mission_mode"></a>
This function is a callback from the service [change_mission_mode](./services/change_mission_mode.html).

The function checks if the new mission mode is known and changes to the new mode.
If the mode is not known it returns False and does nothing

- params
  - request
    - asv_mode: value (int) of the mission mode we want to change into
- output
  - response
    - success: True upon success False otherwise
- managed variables
  - <a href="#self.mission_mode">mission_mode</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET NEXT WAYPOINT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_next_wp()</H3>
<a id="get_next_wp"></a>
This function pops a samplepoint from the list of points
- output
  - next sample point in mission list
- managed variables
  - <a href="#self.samplepoints">samplepoints</a>

<FONT COLOR="#ff0000"> TODO:<br>
- Deprecate MQTT from this function  <br>
- return false if list is empty</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->