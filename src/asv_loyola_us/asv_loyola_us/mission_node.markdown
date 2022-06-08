---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Mission Node</H1> 

This is the main node of the ASV. It contains the <a href="#main">state machine(value)</a> of the robot and the main services that an user will use.
![diagram](../../../assets/main.jpg)

<pre>
Services
/<a href="./services/cancel_movement.html">cancel_movement</a>  <a href="#cancel_movement" style="float:right;text-align:right;">cancel_movement_callback</a>
/<a href="./services/close_asv.html">close_asv</a>  <a href="#close_asv" style="float:right;text-align:right;">close_asv_callback</a>
/<a href="./services/load_mission.html">load_mission</a> <a href="#load_mission" style="float:right;text-align:right;">load_mission_callback</a>
/<a href="./services/change_mission_mode.html">change_mission_mode</a> <a href="#new_mission_mode" style="float:right;text-align:right;">change_mission_mode_callback</a>
/<a href="./services/new_samplepoint.html">new_samplepoint</a> <a href="#new_samplepoint_callback" style="float:right;text-align:right;">new_samplepoint_callback</a>

</pre>
<pre>
Topics
/<a href="./topics/destination.html">destination</a>  <a href="#destination" style="float:right;text-align:right;">destination</a>
/<a href="./topics/mission_mode.html">mission_mode</a>  <a href="#mission_mode" style="float:right;text-align:right;">mission_mode</a>
</pre>
<pre>
Actions
/
</pre>

<pre>
functions
<a href="#arm_vehicle">arm_vehicle(value)</a>
<a href="#cancel_movement_callback">cancel_movement_callback(request, response)</a>
<a href="#change_ASV_mode">change_ASV_mode(mode)</a>
<a href="#change_current_mission_mode">change_current_mission_mode(desired_mode)</a>
<a href="#close_asv_callback">close_asv_callback(request, response)</a>
<a href="#get_next_wp">get_next_wp()</a>
<a href="#go_to">go_to(location)</a>
<a href="#goto_response_callback">goto_response_callback(future)</a>
<a href="#goto_feedback_callback">go_to_feedback_callback(feedback)</a>
<a href="#goto_finished_callback">goto_finished_callback(future)</a>
<a href="#load_mission_callback">load_mission_callback(request, response)</a>
<a href="#main">main()</a>
<a href="#mission_mode_publish">mission_mode_publish()</a>
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
<a id="self.samplepoints">waiting_for_action</a>
<a id="self.samplepoints">current_mission_mode</a>
<a id="self.samplepoints">mission_mode</a>
<a id="self.samplepoints">status</a>
<a id="self.samplepoints">point_backup</a>
</pre>

<pre>
Parameters
<a href="./parameters/mission_filepath.html">mission_filepath</a>
<a href="./parameters/debug.html">debug</a>
<a href="./parameters/simple_goto_timeout.html">simple_goto_timeout</a>
</pre>


<FONT COLOR="#0000ff"> Future ideas:<br>
- buffer of MQTT samplepoints printing them in the server, to create mission (on the air)<br>
</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ARM VEHICLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>arm_vehicle(value) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L343" style="float:right;text-align:right;">code</a></H3>
<a id="arm_vehicle"></a>
This function calls the service [arm_vehicle](./404) to arm the vehicle
- params
  - value: True to arm, false to disarm
- output
  - True when finished


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CANCEL MOVEMENT CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>cancel_movement_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L461" style="float:right;text-align:right;">code</a></H3> 
<a id="cancel_movement_callback"></a>

This function is a callback from the service [/cancel_movement](./404)
This function forces the stop of the drone and deletes the destination point, returning the drone to standby mode






<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE ASV MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_ASV_mode(mode) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L383" style="float:right;text-align:right;">code</a></H3>
<a id="change_ASV_mode"></a>

this function calls the service /[change_asv_mode](./404) to change the mode in which the ardupilot is working

- params
  - mode : value (int or string) of the mode we want the pilot to change into (example: "GUIDED")
- output
    - True upon success False otherwise


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE CURRENT MISSION MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>change_current_mission_mode(desired_mode) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L294" style="float:right;text-align:right;">code</a></H3>
<a id="change_current_mission_mode"></a>

This function is called at the start of each state of the state machine, its used to detect if the mode has changed to run specific sections of code that will initialize the new state

- params
  - desired_mode: value (int) of the mission mode we want to change into
- output
  - True upon current_mission_mode != mission_mode
- managed variables
  - <a href="#self.current_mission_mode">current_mission_mode</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CLOSE ASV CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>close_asv_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L327" style="float:right;text-align:right;">code</a></H3>
<a id="close_asv_callback"></a>

This function is a callback from service /close_asv
This functions turns off the ASV safelly, killing the node
Args:
    bool always true to turn off the vehicle
Returns:
    Bool indicating success of the action

<FONT COLOR="#ff0000"> TODO:<br>
- close the ASV safelly<br>
- may be use as input a value that will try to close it, and other that will force it
- This function is not used, and close is forced in MQTT
</FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET NEXT WAYPOINT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_next_wp() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L259" style="float:right;text-align:right;">code</a></H3> 
<a id="get_next_wp"></a>
This function pops a samplepoint from the list of points if current mission mode is 2, raise an error if that is not the mode
- output
  - next sample point in mission list
- managed variables
  - <a href="#self.samplepoints">samplepoints</a>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOAD MISSION CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>load_mission_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L360" style="float:right;text-align:right;">code</a></H3> 
<a id="load_mission_callback"></a>
This function loads a mission from file if it exists

- params
  - request
    - file_name: (str): name of the file (it must be a number), empty string "" to load last mission
- output
  - response
    - success: True upon success False otherwise
- managed variables
  - <a href="#self.samplepoints">samplepoints</a>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>go_to(location) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L417" style="float:right;text-align:right;">code</a></H3> 
<a id="go_to"></a>
This function starts the call to the action /goto, generating a future

- params
  - location: GPS coordinates where to take a sample

goto_feedback_callback


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO RESPONSE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_response(future) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L417" style="float:right;text-align:right;">code</a></H3> 
<a id="goto_response"></a>
This function is the first answer from the action service, indicates whether the request is accepted or rejected

- params
  - future: either accept or reject

If the point is accepted it logs and acknoledges response to start action

if it is rejected it logs and:
- If we are in mission or simplepoint mode we will retry as ekf may have failed
- If we are in manual mode we will store the point as it means manual interruption through RC
- If we are in any other mode we will do nothing as action must die


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO FEEDBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_feedback_callback(feedback) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L417" style="float:right;text-align:right;">code</a></H3> 
<a id="goto_feedback_callback"></a>
This function prints the feedback from the goto action

- params
  - feedback: distance to the point




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO FINISHED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>goto_finished_callback(future) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L417" style="float:right;text-align:right;">code</a></H3> 
<a id="goto_finished_callback"></a>
This function prints the result once the action finishes

- params
  - future: boolean value, meaningless


it logs whether action finished successfuly or unsuccessfuly and resumes state machine by setting <a href="#waiting_for_action">waiting for action</a> to false


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>main() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L142" style="float:right;text-align:right;">code</a></H3>
<a id="main"></a>

This function contains all the states of the robot. It is executed at a period of 10Hz

![state_machine](../../../miscelaneous/UML_drone.png)

 <ol start="0">
  <li>LAND</li>
  <dd>- Initial state, the robot is disarmed, it warns if vehicle is armed externally</dd>


  <li>Standby</li>
  <dd>- waiting state, the robot is armed in loiter and waiting for signals</dd>
  <dd>- if EKF fails at start it will go into manual mode</dd>

  <li>Pre-loaded mission</li>
  <dd>- if EKF fails at start it will go into manual mode</dd>
  <dd>- checks if there is a mission loaded, going to Standby otherwise</dd>
  <dd>- arms vehicle</dd>
  <dd>- call go_to service while there are points left</dd>
  <dd>- Indicates the mission have finished and changes to Standby mode</dd>

<li>Manual mode</li>
  <dd>- ASV mode changes to Manual and Vehicle is armed</dd>

<li>Simple Go-To</li>
  <dd>- if EKF fails at start it will go into manual mode</dd>
  <dd>- Sets vehicle into Loiter mode</dd>
  <dd>- Arm vehicle</dd>
  <dd>- Waits for points to be received (there is a buffer of 1 point while we are reaching another)</dd>

<li>RTL</li>
  <dd>- change vehicle mode to "RTL"</dd>
</ol> 

It logs an error if mode is different than the ones listed here

- managed variables
  - <a href="#self.mission_mode">mission_mode</a>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MISSION MODE PUBLISH%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>mission_mode_publish() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L259" style="float:right;text-align:right;">code</a></H3> 
<a id="mission_mode_publish"></a>

This function is executed periodically each second indicating mission mode in the topic /<a href="./topics/mission_mode.html">mission_mode</a>.


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW MISSION MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>new_mission_mode(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L259" style="float:right;text-align:right;">code</a></H3> 
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

<H3>new_samplepoint_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L240" style="float:right;text-align:right;">code</a></H3>
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

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>startup() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L102" style="float:right;text-align:right;">code</a></H3>
<a id="startup"></a>

This function checks and waits until every component is running
Managed variables are initialized

- initialized variables
  - <a href="#self.mission_mode">mission_mode: 0</a>
  - <a href="#self.current_mission_mode">current_mission_mode: -1</a>
  - <a href="#self.mission_mode_strs">mission_mode_strs: ["LAND", "STANDBY", "PRELOADED_MISSION", "MANUAL", "SIMPLE POINT", "RTL"]</a>
  - <a href="#self.mqtt_waypoint">mqtt_waypoint: None</a>
  - <a href="#self.status">status: Status()</a>
  - <a href="#self.waiting_for_action">waiting_for_action: False</a>
  - <a href="#self.samplepoints">samplepoints: mission_2</a>
 
<FONT COLOR="#ff0000"> TODO:<br>
- Check elements to add to the list, right now only communication with drone is checked</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS SUBSCRIBER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mission_node.py#L311" style="float:right;text-align:right;">code</a></H3>
<a id="status_suscriber_callback"></a>
This function subscribes to the [ASV_status](./404) topic and updates the status of the ASV

- managed variables
  - <a href="#self.status">status</a>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WATCHDOG CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>watchdog_callback(exception) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/mission_node.py#L308" style="float:right;text-align:right;">code</a></H3>
<a id="watchdog_callback"></a>

This function is run as an except to the code.
If there has been an error due to code this function will run and send the complete info about the error to the watchdog node

  
<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->
