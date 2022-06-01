---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Watchdog Node</H1> 
This node is in charge of raising errors if something is not working properly. must supervise every other node


<FONT COLOR="#ff0000"> TODO:<br>
- Logging, parsing and other errors handling need to be done</FONT>

<pre>
Services
</pre>

<pre>
Topics
/<a href="./topics/error.html">error</a>  <a href="#error_log_publish" style="float:right;text-align:right;">error_log_publish</a>
</pre>

<pre>
functions
<a href="#error_callback">error_callback(msg)</a>
<a href="#error_log_publish">error_log_publish()</a>
<a href="#parse_time">parse_time(time)</a>
<a href="#parse_error">parse_error(error)</a>
<a href="#watchdog_timer_function">watchdog_timer_function()</a>
<a href="#watchdog_callback">watchdog_callback(msg)</a>
</pre>


<pre>
variables
<a id="self.error_list">error_list</a>
<a id="self.last_error_subscriber_number">last_error_subscriber_number</a>
<FONT COLOR="#ff0000"> TODO:
- add watchdog second counters</FONT>
</pre>

<pre>
Parameters
</pre>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ERROR  CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>error_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L31" style="float:right;text-align:right;">code</a></H3>
<a id="error_callback"></a>

This function receives an error that happened in another node through the topic [_internal_error](./404)
it will publish the error in /rosout and store it in the error list
- params
  - msg
    - node: name of the node that had the error
    - message: error that happened
- managed variables
  - <a href="#error_list">error_list(msg)</a>

<FONT COLOR="#ff0000"> TODO:
- it must act and call services, restart nodes or system according to defined routines to fix errors (it must be able to OTA, record version and log data not logged in ros logs (you can observe it in the /rosout topic (for example, line parent function etc)))
- if drone doesnt answer for long enough it will automatically return to a previous working version
</FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ERROR  LOG PUBLISH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>error_log_publish() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L63" style="float:right;text-align:right;">code</a></H3>
<a id="error_log_publish"></a>

Once someone subscribes to the [/error](./404) topic, it will publish all the errors stored in the topic 

<FONT COLOR="#ff0000"> TODO:
- This has been deprecated as this information is carried via MQTT</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARSE TIME %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>parse_time(time) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L44" style="float:right;text-align:right;">code</a></H3>
<a id="parse_time"></a>

To clean the code, it calculates the current time (UTC+1)
<FONT COLOR="#ff0000"> TODO:<br>
- in development, works but its not used</FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARSE ERROR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>parse_error(error) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L48" style="float:right;text-align:right;">code</a></H3>
<a id="parse_error"></a>

Parses the error to make it easier to read

<FONT COLOR="#ff0000"> TODO:<br>
- In development, should also be used to classify errors </FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WATCHDOG TIMER FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>watchdog_timer_function() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L52" style="float:right;text-align:right;">code</a></H3>
<a id="watchdog_timer_function"></a>

Increases the counter of seconds a node has not answered by 1, and raises a subrutine if the number of seconds is higher than X

<FONT COLOR="#ff0000"> TODO:<br>
- In development, used to check if a node is blocked </FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WATCHDOG  CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>watchdog_callback(msg) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/watchdog_node.py#L58" style="float:right;text-align:right;">code</a></H3>
<a id="watchdog_callback"></a>

Resets the counter of the node

<FONT COLOR="#ff0000"> TODO:<br>
- In development </FONT>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->
