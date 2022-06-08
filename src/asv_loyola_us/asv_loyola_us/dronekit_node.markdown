---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Dronekit Node</H1> 

This node is in charge of the communication with the Raspberry pi

![diagram](../../../assets/dronekit.jpg)

<pre>
Services
/<a href="./services/arm_vehicle.html">arm_vehicle</a>  <a href="#arm_vehicle_callback" style="float:right;text-align:right;">arm_vehicle_callback</a>
/<a href="./services/change_asv_mode.html">change_asv_mode</a> <a href="#change_asv_mode_callback" style="float:right;text-align:right;">change_asv_mode_callback</a>
/<a href="./services/reset_home.html">reset_home</a> <a href="#reset_home_callback" style="float:right;text-align:right;">reset_home_callback</a>
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
<a href="#camera_obstacles_callback">camera_obstacles_callback(msg)</a>
<a href="#cancel_sensor_read">cancel_sensor_read()</a>
<a href="#condition_yaw">condition_yaw(heading, relative=false)</a>
<a href="#dictionary">dictionary(dictionary)</a>
<a href="#get_bearing">get_bearing(location2)</a>
<a href="#goto_accept">goto_accept(goal_request)</a>
<a href="#goto_accepted_callback">goto_accepted_callback(goal_handle)</a>
<a href="#goto_cancel">goto_cancel(goal_handle)</a>
<a href="#goto_execute_callback">goto_execute_callback(goal_handle)</a>
<a href="#reached_position">reached_position(current_loc, goal_loc)</a>
<a href="#reset_home_callback">reset_home_callback()</a>
<a href="#rc_read_callback">rc_read_callback()</a>
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

<FONT COLOR="#0000ff"> Future ideas:<br>
- Program a flag in ardupilot if connection with navio is lost to regain connection<br>
- Implement hard reset of navio (may be a terminal handler that sends through ssh a `sudo systemctl restart ardurover`)<br>
</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ARM VEHICLE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>arm_vehicle_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L129" style="float:right;text-align:right;">code</a></H3>
<a id="arm_vehicle_callback"></a>
This function arms or disarms the vehicle
- params
  - request
    - value: True to arm, false to disarm
- output
  - request:
    - success:True if success, False otherwise




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CANCEL SENSOR READ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>cancel_sensor_read() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L519" style="float:right;text-align:right;">code</a></H3>
<a id="calculate_distance"></a>

Is just a handler that candels the action read_sensor.



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE ASV MODE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_asv_mode_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L264" style="float:right;text-align:right;">code</a></H3>
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

<H3>condition_yaw(heading, relative) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L199" style="float:right;text-align:right;">code</a></H3>
<a id="condition_yaw"></a>

<FONT COLOR="#ff0000"> TODO:<br>
- THIS FUNCTION HAS BEEN DEPRECATED IN VERSION 1.2.0</FONT>

This function send a MAV_CMD_CONDITION_YAW command through pymavlink to specify a heading to take when approaching a point
This function is used due to the [issue](https://github.com/diydrones/ardupilot/issues/2427), you can find more info in [ardupilot wiki](http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw)

- parameters:
  - heading: angle in degrees to take in yaw
  - relative: flag to take the heading value in absolute value or with a relative offset



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CALCULATE DISTANCE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>calculate_distance(goal_loc) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L237" style="float:right;text-align:right;">code</a></H3>
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

<H3>get_bearing(location1, location2) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L179" style="float:right;text-align:right;">code</a></H3>
<a id="get_bearing"></a>

Returns the bearing between the two LocationGlobal objects passed as parameters.
This method is an approximation, and may not be accurate over large distances and close to the earth's poles. It comes from the [ArduPilot test code](https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py`):

- params
  - location1: Actual position
  - location2: Reference position
- output
  - bearing: the angle difference from location1 to location2

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO ACCEPT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_accept(goal_request) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L290" style="float:right;text-align:right;">code</a></H3>
<a id="goto_accept"></a>

This function manages action calls it will reject the call if:
- There is another call in progress
- Vehicle is not armed or vehicle mode is not loiter
- Distance to destination is greater than 5km

Args:
- goal_request:
  - request:

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO ACCEPTED CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_accepted_callback(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L308" style="float:right;text-align:right;">code</a></H3>
<a id="goto_accepted_callback"></a>

declares variables to indicate an action is in process
If the destination point is too far away (default 5km) it will finish the action before the execution phase

Args:
- goal_handle:
  - request:
    - samplepoint: (Location) destination where to take a sample




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO CANCEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_cancel(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L318" style="float:right;text-align:right;">code</a></H3>
<a id="goto_cancel"></a>

This function manages action canceled, it only logs
Args:
- goal_handle:
  - request:
    - samplepoint: (Location) destination where to take a sample


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO EXECUTE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_execute_callback(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L324" style="float:right;text-align:right;">code</a></H3>
<a id="goto_execute_callback"></a>

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
- Cancel was not requested
- RC is in auto mode (exit action otherwise)
- EKF is ok, waiting in manual mode disarmed while EKF is off 
- mode is not guided (change to guided), log if ekf was fixed
- ASV is disarmed (This means RC interrupted mission), wait for RC positions to go back to normal (waiting state).

There is a counter in this process so if ship invest more than 1 minute to reach point it will leave

Once samplepoint is reached, change into loiter mode and call action take sample.
- If RC changes it will force close
- If action is cancelled it will finish action

After sensor read finishes it will finish and return success





<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REACHED POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->
<H3>reached_position(goal_loc) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L227" style="float:right;text-align:right;">code</a></H3>
<a id="reached_position"></a>
This function returns true if the goal_loc is withing 1.5m of the drone, false otherwise.

- params
  - goal_loc: reference position
- output
  - True if the waypoint is within 0.5 meters to the target waypoint



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS PUBLISH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_publish(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L154" style="float:right;text-align:right;">code</a></H3>
<a id="status_publish"></a>

This function publishes the [status](./404) (battery, longitude, latitude, yaw, vehicle id, armed_status) at a rate of 2Hz on the topic [/status](./404)


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RC READ CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>rc_read_callback() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L479" style="float:right;text-align:right;">code</a></H3>
<a id="rc_read_callback"></a>

This function goberns the RC reads from channels.

It works as a the second level of the RC control. Ardupilot reads RC but just acts after a change in PWM read.


- If channel read is 0, rc is disconnected,
<FONT COLOR="#999900"> WARNING:<br>
This funciton is not properly done, as values are 0 if RC was never connected and last value if its connected (depends on the RC used)</FONT>

- if channel 6 is high we are in manual interruption mode
- if channel 5 is high armed is on


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS PUBLISH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_publish(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L154" style="float:right;text-align:right;">code</a></H3>
<a id="status_publish"></a>

This function publishes the [status](./404) (battery, longitude, latitude, yaw, vehicle id, armed_status) at a rate of 2Hz on the topic [/status](./404)


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SENSOR READ FINISHED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>sensor_read_finished(future) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L548" style="float:right;text-align:right;">code</a></H3>
<a id="camera_obstacle_callback"></a>

Callback of the end of the sensor read action. It returns the sensor reads and raises the flag indicating that sensor read has finished.

-params:
  -future:
    - sensor_reads: array of all sensor reads
    - finish_flag: String indicating finish reason


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA OBSTACLE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>camera_obstacle_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/dronekit_node.py#L548" style="float:right;text-align:right;">code</a></H3>
<a id="camera_obstacle_callback"></a>

This function is suscribed to the topic <a href="/topics/camera_obstacle.html">/camera_obstacle</a>, it parses the message and sends it to ardusub so it includes it in its map and evades obstacles

-params:
  -msg:
    - distance: array indicating distance to the obstacle in cm
    - angle_increment: array indicating the angle occupied by the obstacle (used 1.5º by default)

<FONT COLOR="#999900"> WARNING:<br>
TODO</FONT>




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


