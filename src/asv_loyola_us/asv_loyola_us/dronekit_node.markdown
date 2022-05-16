---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Dronekit Node</H1> 

This node is in charge of the communication with the Raspberry pi

![diagram](../../../assets/dronekit.jpg)

<FONT COLOR="#ff0000"> TODO:<br>
- The name of the node will be changed to communication when able to<br>
- The communication must be translated to pymavlink deprecating dronekit to avoid errors and simplify communication as dronekit is way too much transparent</FONT>

<pre>
Services
/<a href="./services/arm_vehicle.html">arm_vehicle</a>  <a href="#arm_vehicle_callback" style="float:right;text-align:right;">arm_vehicle_callback</a>
/<a href="./services/change_asv_mode.html">change_asv_mode</a> <a href="#change_asv_mode_callback" style="float:right;text-align:right;">change_asv_mode_callback</a>
</pre>

<pre>
Topics
/<a href="./topics/status.html">status</a>  <a href="#status_publish" style="float:right;text-align:right;">status_publish</a>
</pre>

<pre>
Actions
/<a href="./actions/go_to.html">go_to_point_command</a> <a href="#go_to_point_callback" style="float:right;text-align:right;">go_to_point_callback</a>
</pre>

<pre>
functions
<a href="#arm_vehicle_callback">arm_vehicle_callback(request, response)</a>
<a href="#change_asv_mode_callback">change_asv_mode_callback(request, response)</a>
<a href="#calculate_distance">calculate_distance(goal_loc)</a>
<a href="#condition_yaw">condition_yaw(heading, relative=false)</a>
<a href="#dictionary">dictionary(dictionary)</a>
<a href="#get_bearing">get_bearing(location2)</a>
<a href="#goto_accept">goto_accept(goal_request)</a>
<a href="#goto_accepted_callback">goto_accepted_callback(goal_handle)</a>
<a href="#goto_cancel">goto_cancel(goal_handle)</a>
<a href="#goto_execute_callback">goto_execute_callback(goal_handle)</a>
<a href="#reached_position">reached_position(current_loc, goal_loc)</a>
<a href="#status_publish">status_publish()</a>
</pre>


<pre>
variables
<a id="self.status">status</a>
<a id="self.vehicle">vehicle</a>
<a id="self.mode_type">mode_type</a>
<a id="self.goto_goal_handle">goto_goal_handle</a>
</pre>

<pre>
Parameters
<a href="./parameters/vehicle_ip.html">vehicle_ip</a>
<a href="./parameters/timeout.html">timeout</a>
<a href="./parameters/vehicle_id.html">vehicle_id</a>
<a href="./parameters/status.html">status</a>
</pre>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ARM VEHICLE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>arm_vehicle_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L79" style="float:right;text-align:right;">code</a></H3>
<a id="arm_vehicle_callback"></a>
This function arms or disarms the vehicle
- params
  - request
    - value: True to arm, false to disarm
- output
  - request:
    - success:True if success, False otherwise



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE ASV MODE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_asv_mode_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L193" style="float:right;text-align:right;">code</a></H3>
<a id="change_asv_mode_callback"></a>


This functionis a callback from the service [change_asv_mode](./404) changes the mode of the ASV

String value is preferential over int value

- params
  - request
    - asv_mode: value (int) of the mode we want to change into
    - asv_mode_str: value (string) of the mode we want to change into
- output
  - request:
    - success:True if success, False otherwise
- managed variables
  - <a href="#self.vehicle.mode">vehicle.mode</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONDITION YAW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>condition_yaw(heading, relative) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L137" style="float:right;text-align:right;">code</a></H3>
<a id="condition_yaw"></a>

This function send a MAV_CMD_CONDITION_YAW command through pymavlink to specify a heading to take when approaching a point
This function is used due to the [issue](https://github.com/diydrones/ardupilot/issues/2427), you can find more info in [ardupilot wiki](http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw)

- parameters:
  - heading: angle in degrees to take in yaw
  - relative: flag to take the heading value in absolute value or with a relative offset


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CALCULATE DISTANCE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>calculate_distance(goal_loc) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L216" style="float:right;text-align:right;">code</a></H3>
<a id="calculate_distance"></a>

Returns the ground distance in metres towads a Location object.
This method is an approximation, and will not be accurate over large distances and close to the
earth's poles. It comes from the ArduPilot test code:
https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
Args:
    goal_loc: (Location)
Returns:
    distance from the ASV to the goal_loc in meters



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DICTIONARY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>dictionary(dictionary) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L256" style="float:right;text-align:right;">code</a></H3>
<a id="dictionary"></a>

This function is used to load a JSON dictionary for mavlink modes, in order to make code more readable

- params
  - dictionary: dictionary to access

<FONT COLOR="#ff0000"> TODO:<br>
- Search pymavlink library dictionaries instead of creating our own</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET BEARING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_bearing(location1, location2) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L117" style="float:right;text-align:right;">code</a></H3>
<a id="get_bearing"></a>

Returns the bearing between the two LocationGlobal objects passed as parameters.
This method is an approximation, and may not be accurate over large distances and close to the earth's poles. It comes from the [ArduPilot test code](https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py`):

- params
  - location1: Actual position
  - location2: Reference position
- output
  - bearing: the angle difference from location1 to location2

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO ACCEPT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_accept(goal_request) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L286" style="float:right;text-align:right;">code</a></H3>
<a id="goto_accept"></a>

This function manages action calls it will reject the call if:
- There is another call in progress
- Vehicle is not armed or vehicle mode is not loiter
- Distance to destination is greater than 5km

Args:
- goal_request:
  - request:
    - samplepoint: (Location) destination where to take a sample


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO ACCEPTED CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_accepted_callback(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L308" style="float:right;text-align:right;">code</a></H3>
<a id="goto_accepted_callback"></a>

declares variables to indicate an action is in process

Args:
- goal_handle:
  - request:
    - samplepoint: (Location) destination where to take a sample




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO CANCEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_cancel(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L286" style="float:right;text-align:right;">code</a></H3>
<a id="goto_cancel"></a>

This function manages action canceled, it only logs
Args:
- goal_handle:
  - request:
    - samplepoint: (Location) destination where to take a sample


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO EXECUTE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_cancel(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L286" style="float:right;text-align:right;">code</a></H3>
<a id="goto_cancel"></a>

This function goberns the go to behaviour

Args:
- goal_handle:
  - request:
    - samplepoint: (Location) destination where to take a sample
  - feedback:
    - distance: (float) distance to waypoint
  - response:
    -success: (Bool)


It starts calling planner to get the path to follow and extracting it to a variable, if there is no planner it will log and finish action

While there are points left it will go to point, while going to point it will check:
- EKF is ok, waiting in manual mode while EKF is off
- mode is not guided (change to guided), log if ekf was fixed
- ASV is disarmed (This means RC interrupted mission) log and switch to manual mode, finishing action but not erasing mission, to be able to resume

Once samplepoint is reached, change into loiter mode and take sample, then return success





<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REACHED POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->
<H3>reached_position(goal_loc) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L165" style="float:right;text-align:right;">code</a></H3>
<a id="reached_position"></a>
This function returns true if the goal_loc is withing 1.5m of the drone, false otherwise.

- params
  - current_loc: actual position
  - goal_loc: reference position
- output
  - True if the waypoint is within 0.5 meters to the target waypoint
  

<FONT COLOR="#ff0000"> TODO:<br>
- There is no need to use the current_loc param</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS PUBLISH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_publish(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L102" style="float:right;text-align:right;">code</a></H3>
<a id="status_publish"></a>

This function publishes the [status](./404) (battery, longitude, latitude, yaw, vehicle id, armed_status) at a rate of 2Hz on the topic [/status](./404)

<FONT COLOR="#999900"> WARNING:<br>
- This will raise an error if the data read from the drone is of different type, the data must be of the type Status() variables need.<br>
- If data is invalid it will publish a -1 value (value imposible to give normally)</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


