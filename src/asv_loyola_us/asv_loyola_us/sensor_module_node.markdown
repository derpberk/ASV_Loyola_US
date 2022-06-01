---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Sensor module Node</H1> 


This is the sensor node of the ASV. 

![diagram](../../../assets/sensors.jpg)

<pre>
Services
/<a href="./services/Sensor_params.html">Sensor_params</a>  <a href="#update_parameters_callback" style="float:right;text-align:right;">update_parameters_callback</a>
</pre>
<pre>
Topics
/<a href="./services/sensors.html">sensors</a>
</pre>
<pre>
Actions
<a href="./actions/sensor_read.html">sensor_read</a>  <a href="#sensor_read_callback" style="float:right;text-align:right;">sensor_read_action</a>
</pre>

<pre>
functions
<a href="#get_sample_callback">get_sample_callback(request, response)</a>
<a href="#read_sensor">read_sensor()</a>
<a href="#status_suscriber_callback">status_suscriber_callback(msg)</a>
</pre>


<pre>
variables
<a id="self.is_frame_ok">is_frame_ok</a>
<a id="self.sensor_data">sensor_data</a>
<a id="self.sensor_goal_handle">sensor_goal_handle</a>
<a id="waited_time">waited_time</a>
</pre>

<pre>
Parameters
<a href="./parameters/baudrate.html">baudrate</a>
<a href="./parameters/debug.html">debug</a>
<a href="./parameters/num_samples.html">num_samples</a>
<a href="./parameters/pump.html">pump</a>
<a href="./parameters/pump_channel.html">pump_channel</a>
<a href="./parameters/sensor_read_timeout.html">sensor_read_timeout</a>
<a href="./parameters/timeout.html">timeout</a> 
<a href="./parameters/time_between_samples.html">time_between_samples</a>
<a href="./parameters/USB_string.html">USB_string</a>ç
</pre>
<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET SAMPLE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_sample_callback() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L223" style="float:right;text-align:right;">code</a></H3>
<a id="sensor_read_callback"></a>

This function is a callback from the action <a href="./actions/sensor_read.html">/sensor_read</a>. It takes the number of samples specified in the <a href="./parameters/num_samples.html">num_samples</a> parameter. 

- params
  - debug:
    - True — Take a debug value
    - False — Take a real sample from the Smart Water
- output
  - sensor_reads: <a href="./services/get_sample.html">Sensor[]</a> data type indicating all sensor reads
- feedback
  - sensor_read: <a href="./services/get_sample.html">Sensor</a> data type indicating sensor read

As it is an action it has the structure of one <a id="../../asv_interfaces/asv_interfaces.html">check guide if needed</a>. The acknowledge and accept phase are just a log. The execution phase calls read sensor.

If specified it will also activate the pump when needed.

after each read it will check if it has been canceled

<FONT COLOR="#ff0000"> TODO:<br>
- If ever needed more real time, send goal_handle as argument to <a href="#read_sensor">read_sensor()</a> or execute it in a thread </FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% READ SENSOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>read_sensor() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L100" style="float:right;text-align:right;">code</a></H3>
<a id="read_sensor"></a>

This function Reads from the serial port of the Smart Water and parses the message into <a href="self.sensor_data">sensor_data</a>.
it has a timeout indicated in <a href="./parameters/sensor_read_timeout.html">
It also checks whether the value read is correct.

After a read it will publish it to <a href="./services/sensors.html">/sensors</a> topic, so that MQTT sends it to server.

If a manual iterruption occurs it will exit


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE PARAMETERS CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>update_parameters_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L190" style="float:right;text-align:right;">code</a></H3>
<a id="update_parameters_callback"></a>

This function updates the value of the parameters logging changes

- request
  - read_only: (Bool) If true parameters wont be updated
  - pump_channel: (Int)
  - number_of_samples: (int)
  - time_between_samples: (float)
  - use_pump: (Bool)

- response
  - pump_channel: (Int)
  - number_of_samples: (int)
  - time_between_samples: (float)
  - use_pump: (Bool)

  
<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS SUBSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L187" style="float:right;text-align:right;">code</a></H3>
<a id="status_suscriber_callback"></a>

This function is a suscriber of /<a href="./topics/status.html">status</a> topic, it stores the variables indicating the status of the drone