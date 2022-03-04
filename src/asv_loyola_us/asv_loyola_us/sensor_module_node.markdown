---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Sensor module Node</H1> 


This is the planner node of the ASV. 


<pre>
Services
/<a href="./services/close_asv.html">close_asv</a>  <a href="#" style="float:right;text-align:right;">close_asv_callback</a>
/<a href="./services/load_mission.html">load_mission</a> <a href="#load_mission" style="float:right;text-align:right;">load_mission_callback</a>
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
<a href="#load_mission_callback">load_mission_callback(request, response)</a>
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
