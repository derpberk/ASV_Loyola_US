---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>MQTT Node</H1> 

This node is in charge or the communications with the server

![diagram](../../../assets/mqtt.jpg)

<pre>
Services
/<a href="./services/MQTT_send_info.html">MQTT_send_info</a>  <a href="#MQTT_send_info" style="float:right;text-align:right;">MQTT_send_info</a>
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
MQTT messages
/<a href="./MQTT_messages/mission_type.html">mission_type</a>  <a href="#MQTT_messages" style="float:right;text-align:right;">messages</a>
/<a href="./MQTT_messages/load_mission.html">load_mission</a> 
/<a href="./MQTT_messages/cancel_movement.html">cancel_movement</a> 
/<a href="./MQTT_messages/update_parameters.html">update_parameters</a> 
/<a href="./MQTT_messages/read_params.html">read_params</a> 
/<a href="./MQTT_messages/start_recording.html">start_recording</a> 
/<a href="./MQTT_messages/stop_recording.html">stop_recording</a> 
/<a href="./MQTT_messages/obstacle_avoidance.html">obstacle_avoidance</a> 
/<a href="./MQTT_messages/reset_home.html">reset_home</a> 
</pre>

<pre>
functions
<a href="#asv_send_info">asv_send_info()</a>
<a href="#destination_subscriber_callback">destination_subscriber_callback(msg)</a>
<a href="#log_subscriber_callback">log_subscriber_callback(msg)</a>
<a href="#main">main()</a>
<a href="#mission_mode_suscriber_callback">mission_mode_suscriber_callback(msg)</a>
<a href="#on_message">on_message(client, _, message)</a>
<a href="#on_disconnect">on_disconnect(client, _, message)</a>
<a href="#params_read">params_read()</a>
<a href="#sensors_subscriber_callback">sensors_subscriber_callback(msg)</a>
<a href="#sendinfo_callback">send_info_callback(request, response)</a>
<a href="#shutdown_asv">shutdown_asv()</a>
<a href="#status_suscriber_callback">status_suscriber_callback(msg)</a>
</pre>

<pre>
variables
<a id="self.processing">processing</a>
<a id="self.mission_mode">mission_mode</a>
<a id="self.status">status</a>
<a id="self.mqtt_point">mqtt_point</a>
<a id="self.call_msg">call_msg</a>
<a id="self.shutdown">shutdown</a>
<a id="self.camera_signal">camera_signal</a>
<a id="self.update_params">update_params</a>
<a id="self.read_params">read_params</a>
<a id="self.camera_handler">camera_handler</a>
<a id="self.enable_planner">enable_planner</a>
<a id="self.sensor_params">sensor_params</a>
<a id="self.cancel_movement">cancel_movement</a>
</pre>

<pre>
Parameters
<a href="./parameters/vehicle_id.html">vehicle_id</a>
<a href="./parameters/mqtt_addr.html">mqtt_addr</a>
<a href="./parameters/internet_loss_timeout.html">internet_loss_timeout</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ASV_SEND_INFO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>asv_send_info() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L109" style="float:right;text-align:right;">code</a></H3>
<a id="asv_send_info"></a>
This function is executed periodically each 0.5 seconds by a timer, it pubblishes in the server the following information about the status of the ASV.
- Location
- Heading
- Vehicle number
- Battery
- Arm status
- Mission mode
- ASV mode
- EKF_status
- Manual_switch_status


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DESTINATION SUSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>destination_subscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L212" style="float:right;text-align:right;">code</a></H3>
<a id="destination_subscriber_callback"></a>
This function is a suscriber of /<a href="./topics/destination.html">destination</a> topic, it sends a mqtt message to the topic "destination" indicating:
- vehicle number
- location

The server interprets this data as the place where to put a red circle, indicating the point where the ASV wants to take a sample


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOG SUSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>log_subscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L195" style="float:right;text-align:right;">code</a></H3>
<a id="log_subscriber_callback"></a>

This function is a suscriber of /<a href="./topics/rosout.html">rosout</a> topic, it sends a mqtt message to the topic "log" indicating:
- vehicle number
- origin node
- time of the message
- log message



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>main() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L88" style="float:right;text-align:right;">code</a></H3>
<a id="main"></a>

This function declares variables and tells server that drone is starting, after that its main behaviour consist in a loop that calls a service when it is necessary


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MISSION MODE SUBSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>mission_mode_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L192" style="float:right;text-align:right;">code</a></H3>
<a id="mission_mode_suscriber_callback"></a>

This function is a suscriber of /<a href="./topics/mission_mode.html">mission_mode</a> topic, it stores the string indicating the mission mode




<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ON MESSAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>on_message(_client, _, msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L130" style="float:right;text-align:right;">code</a></H3>
<a id="on_message"></a>
<a id="MQTT_messages"></a>
Asyncronous handler of a MQTT message. Ir receives a message from the broker. Depending on the fields of the input message, setting the variables so that the main loop executes the call services.

This function is kept independent of the main loop and executed in a thread so it is kept alive even if ROS2 Crashes to execute a hard reset remotely

It uses a variable <a id="self.processing">processing</a>  to avoid massive input of messages.

-Args:
 - _client: Client object
 - msg: MQTT message object.



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ON DISCONNECT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>on_disconnect(_client, _, msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L130" style="float:right;text-align:right;">code</a></H3>
<a id="on_disconnect"></a>

This function logs if MQTT connection closes and internet status (no action required as loop_forever() is enabled (connection is automatically recovered))

-Args:
 - _client: Client object
 - msg: MQTT message object.

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SEND INFO CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>sendinfo_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L174" style="float:right;text-align:right;">code</a></H3>
<a id="sendinfo_callback"></a>
This function starts or stops the timer of <a href="#asv_send_info">asv_send_info()</a> and is a callback from the service <a href="./services/MQTT_send_info.html">/MQTT_send_info</a>

-Args:
 - request
  - value: 
   - True — Start sending
   - False — Stop sending
 - response:
  - success:
   - True — upon success
   - False — otherwise





<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensors subscriber callback %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>sensors_subscriber_callback() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L220" style="float:right;text-align:right;">code</a></H3>
<a id="sensors_subscriber_callback"></a>

This function is a suscriber of /<a href="./topics/sensors.html">destination</a> topic, it constructs the data sends a mqtt message to the topic "database" indicating if the field exists:

- vehicle number
- location
- date
- ph
- Smart Water battery
- Disolved Oxygen
- Temperature
- Conductivity
- Oxidation Reduction Potential


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SHUTDOWN ASV %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>shutdown_asv() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L342" style="float:right;text-align:right;">code</a></H3>
<a id="shutdown_asv"></a>

this function is callable through MQTT even if ros2 crashes as it is executed in a new thread once shutdown is called

Stops ASV status update, sends info about shutting ros2 to server and waits 2 seconds for other logs to happen, afterwards it hard closes Ros2 and makes <a href=../../startup.html>startup</a> to start again

<FONT COLOR="#ff0000"> TODO:<br>
- If Dronekit or mission node crashes the state of the drone will be unknown, ardupilot may be heading to a different direction.
For more security in the future this must be promoted by watcdog node to regain control of the vehicle and force close it in a secure state before hard reset ros
</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMS READ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>params_read() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L364" style="float:right;text-align:right;">code</a></H3>
<a id="params_read"></a>

This function sends an MQTT message updating the values of the parameters, it parses the service call response



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS SUBSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/mqtt_node.py#L128" style="float:right;text-align:right;">code</a></H3>
<a id="status_suscriber_callback"></a>

This function is a suscriber of /<a href="./topics/status.html">status</a> topic, it stores the variables indicating the status of the drone




