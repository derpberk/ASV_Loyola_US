---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Sensor module Node</H1> 


This is the sensor node of the ASV. 


<pre>
Services
/<a href="./services/get_sample.html">get_sample</a>  <a href="#get_sample_callback" style="float:right;text-align:right;">get_sample_callback</a>
</pre>
<pre>
Topics
/<a href="./services/sensors.html">sensors</a>
</pre>
<pre>
Actions
/
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
<a id="self.samplepoints">samplepoints</a>
</pre>

<pre>
Parameters
<a href="./parameters/baudrate.html">baudrate</a>
<a href="./parameters/debug.html">debug</a>
<a href="./parameters/num_samples.html">num_samples</a>
<a href="./parameters/pump.html">pump</a>
<a href="./parameters/pump_channel.html">pump_channel</a>
<a href="./parameters/timeout.html">timeout</a>
<a href="./parameters/USB_string.html">USB_string</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET SAMPLE CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_sample_callback() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L80" style="float:right;text-align:right;">code</a></H3>
<a id="get_sample_callback"></a>

This function is a callback from the service <a href="./services/get_sample.html">/get_sample</a>. It takes the number of samples specified in the <a href="./parameters/num_samples.html">num_samples</a> parameter. 

- params
  - request
    - debug:
     - True — Take a debug value
     - False — Take a real sample from the Smart Water
- output
  - response
    - sensor: <a href="./services/get_sample.html">Sensor</a> data type indicating sensor reads


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% READ SENSOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>read_sensor() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L120" style="float:right;text-align:right;">code</a></H3>
<a id="read_sensor"></a>

This function Reads from the serial port of the Smart Water and parses the message into variables.


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATUS SUBSCRIBER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>status_suscriber_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/sensor_module_node.py#L310" style="float:right;text-align:right;">code</a></H3>
<a id="status_suscriber_callback"></a>

This function is a suscriber of /<a href="./topics/status.html">status</a> topic, it stores the variables indicating the status of the drone