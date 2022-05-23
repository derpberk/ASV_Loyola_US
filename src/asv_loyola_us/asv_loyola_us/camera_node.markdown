---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Planner Node</H1> 

This node is in charge of the camera

<FONT COLOR="#ff0000"> TODO:<br>
- Check if there is camera at start</FONT>

<pre>
Services
/<a href="./services/camera_recording.html">camera_recording</a>  <a href="#camera_recording" style="float:right;text-align:right;">camera_recording_callback</a>
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
<a href="#status_suscriber_callback">status_suscriber_callback(msg)</a>
<a href="#mission_mode_suscriber_callback">mission_mode_suscriber_callback(msg)</a>
<a href="#camera_recording_callback">camera_recording_callback(request, response)</a>
</pre>


<pre>
variables
<a id="self.path">recording</a>
<a id="self.map">mission_mode</a>
<a id="self.status">status</a>
</pre>

<pre>
Parameters
<a href="./parameters/vehicle_id.html">vehicle_id</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA RECORDING CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>camera_recording_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="camera_recording_callback"></a>

This function manages start and stop recording

-params:
  - request: (bool), true to start recording, false to stop recording

It logs changes

<FONT COLOR="#ff0000"> TODO:<br>
- use pid instead of variable to check if camera is recording for more assurance</FONT>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA OBJECT RECOGNITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>Obstacle_Avoidance()<a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="Obstacle_Avoidance"></a>

This function runs in an independent thread.

the object recognition has few params that can be read
- label
- confidence
- id
- tracking_state
- position
- velocity
- dimensions
- bounding_box_2d
- bounding_box

The object is measured and classified, if it corresponds to an obstacle its data is recovered and sent to the topic <a id="camera_obstacle">/camera_obstacle</a> 

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->