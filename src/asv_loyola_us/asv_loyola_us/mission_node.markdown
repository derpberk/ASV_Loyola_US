---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Mission Node</H1> 

This is the main node of the ASV. It contains the state machine of the robot and the main services that an user will use.


<pre>
Services
/<a href="./services/close_asv.html">close_asv</a>  <a href="#" style="float:right;text-align:right;">close_asv_callback</a>
/<a href="./services/change_mission_mode.html">change_mission_mode</a> <a href="#new_mission_mode" style="float:right;text-align:right;">change_mission_mode_callback</a>
/<a href="./services/new_samplepoint.html">new_samplepoint</a> <a href="#new_samplepoint_callback" style="float:right;text-align:right;">new_samplepoint_callback</a>
</pre>
<pre>
Topics
/
</pre>
<pre>
Actions
/
</pre>

<pre>
functions
<a href="#arm_vehicle">arm_vehicle(value)</a>
<a href="#change_ASV_mode">change_ASV_mode(mode)</a>
<a href="#change_current_mission_mode">change_current_mission_mode(desired_mode)</a>
<a href="#close_asv_callback">close_asv_callback(request, response)</a>
<a href="#get_next_wp">get_next_wp()</a>
<a href="#main">main()</a>
<a href="#new_mission_mode">new_mission_mode(request, response)</a>
<a href="#new_samplepoint_callback">new_samplepoint_callback(request, response)</a>
<a href="#startup">startup()</a>
<a href="#status_suscriber_callback">status_suscriber_callback(msg)</a>
<a href="#watchdog_callback">watchdog_callback(exception)</a>
</pre>


<pre>
variables
<a id="self.mission_mode">mission_mode</a>
<a id="self.samplepoints">samplepoints</a>
</pre>

<pre>
Parameters
<a href="./parameters/mission_filepath.html">mission_filepath</a>
<a href="./parameters/debug.html">debug</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ARM VEHICLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>arm_vehicle(value) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L266" style="float:right;text-align:right;">code</a></H3>
<a id="arm_vehicle"></a>
This function calls the service [arm_vehicle](./404) to arm the vehicle
- params
  - value: True to arm, false to disarm
- output
  - True when finished




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE ASV MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_ASV_mode(mode) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L277" style="float:right;text-align:right;">code</a></H3>
<a id="change_ASV_mode"></a>

this function calls the service [change_asv_mode](./404) to change the mode in which the ardupilot is working

- params
  - mode : value (int or string) of the mode we want the pilot to change into (example: "GUIDED")
- output
    - True upon success False otherwise


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE CURRENT MISSION MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_current_mission_mode(desired_mode) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L238" style="float:right;text-align:right;">code</a></H3>
<a id="change_current_mission_mode"></a>

This function is called at the start of each state of the state machine, its used to detect if the mode has changed to run specific sections of code that will initialize the new state

- params
  - desired_mode: value (int) of the mission mode we want to change into
- output
  - True upon current_mission_mode != mission_mode
- managed variables
  - <a href="#self.current_mission_mode">current_mission_mode</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CLOSE ASV CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>close_asv_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L257" style="float:right;text-align:right;">code</a></H3>
<a id="close_asv_callback"></a>

- params
  - request:
    - value: 
- output
  - response:
    - 
- managed variables
  - 

<FONT COLOR="#ff0000"> TODO:<br>
- close the ASV safelly<br>
- may be use as input a value that will try to close it, and other that will force it
</FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET NEXT WAYPOINT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_next_wp() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L211" style="float:right;text-align:right;">code</a></H3> 
<a id="get_next_wp"></a>
This function pops a samplepoint from the list of points
- output
  - next sample point in mission list
- managed variables
  - <a href="#self.samplepoints">samplepoints</a>
<FONT COLOR="#ff0000"> TODO:<br>
- Deprecate MQTT from this function  <br>
- return false if list is empty</FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>main() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L119" style="float:right;text-align:right;">code</a></H3>
<a id="main"></a>

This function contains all the states of the robot.
 <ol start="0">
  <li>Standby</li>
  <dd>- disarm robot and wait</dd>

  <li>Pre-loaded mission</li>
  <dd>- arm vehicle</dd>
  <dd>- load samplepoint list</dd>
  <dd>- call go_to service while there are points left</dd>
<FONT COLOR="#ff0000">TODO:<br>
    - use planner and collect samples making sure to store them</FONT>

<li>Manual mode</li>
  <dd>- Call service to change ASV mode to "MANUAL"</dd>
<FONT COLOR="#ff0000">TODO:<br>
    - check if vehicle is armed</FONT>

<li>Simple Go-TO</li>
  <dd>- Arm vehicle</dd>
  <dd>- Go to a point and take a sample</dd>
  <dd>- Change to Standby mode</dd>

<li>RTL</li>
  <dd>- change vehicle mode to "RTL"</dd>

</ol> 

It logs an error if mode is different than the ones listed here

- managed variables
  - <a href="#self.mission_mode">mission_mode</a>


<FONT COLOR="#ff0000"> TODO:<br>
- clean and test code  <br>
- Fix state 3 and 4 <br>
- Use a service to reset samplepoint list</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW MISSION MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>new_mission_mode(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L196" style="float:right;text-align:right;">code</a></H3> 
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



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW SAMPLE POINT CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>new_samplepoint_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L179" style="float:right;text-align:right;">code</a></H3>
<a id="new_samplepoint_callback"></a>

This functions recibes a new samplepoint and adds it to the list of points the ASV wants to visit

- params
  - request
    - new_point: location of the next point where we want to take a sample
- output
  - response
    - point_list: array with the points that the ASV has yet to visit
- managed variables
  - <a href="#self.samplepoints">samplepoints[]</a>
  - <a href="#self.mqtt_waypoint">mqtt_waypoint</a>

<FONT COLOR="#ff0000"> TODO:<br>
- Deprecate MQTT waypoint</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>startup() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L63" style="float:right;text-align:right;">code</a></H3>
<a id="startup"></a>

This function checks and waits until every component is running
Managed variables are initialized

- initialized variables
  - <a href="#self.mission_mode">mission_mode: 0</a>
  - <a href="#self.current_mission_mode">current_mission_mode: -1</a>
  - <a href="#self.mission_mode_strs">mission_mode_strs: ["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]</a>
  - <a href="#self.mqtt_waypoint">mqtt_waypoint: []</a>
  - <a href="#self.status">status: Status()</a>
  - <a href="#self.samplepoints">samplepoints: mission_2</a>
 
<FONT COLOR="#ff0000"> TODO:<br>
- Check elements to add to the list, right now only communication with drone is checked</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS SUBSCRIBER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L254" style="float:right;text-align:right;">code</a></H3>
<a id="status_suscriber_callback"></a>
This function subscribes to the [ASV_status](./404) topic and updates the status of the ASV

- managed variables
  - <a href="#self.status">status</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WATCHDOG CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>watchdog_callback(exception) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L308" style="float:right;text-align:right;">code</a></H3>
<a id="watchdog_callback"></a>

This function is run as an except to the code.
If there has been an error due to code this function will run and send the complete info about the error to the watchdog node



<FONT COLOR="#ff0000"> TODO:<br>
- May be find a better way to log things and to store them (for example, use ROS2 logs)</FONT>

  
<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->
