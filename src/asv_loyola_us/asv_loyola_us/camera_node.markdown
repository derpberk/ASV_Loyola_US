---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Planner Node</H1> 

This node is in charge of the camera. Check [camera code](https://github.com/stereolabs/zed-python-api/blob/master/src/pyzed/sl.pyx) or [camera examples](https://github.com/stereolabs/zed-examples) for more information about it.

First step in the code is initialization and configuration.

Camera is set to HD quality 30fps and max precission in depth sensing. Units set to meters.

Afterwards we enable tracking for custom objetcs, position tracking and object detection.
objects stored will be stored in <a href=#self.objects>self.objects</a> variable. The rest parameters will be left at default value.

<pre>
Services
/<a href="./services/camera_recording.html">camera_recording</a>  <a href="#camera_recording" style="float:right;text-align:right;">camera_recording_callback</a>
/<a href="./services/enable_obstacle_avoidance.html">enable_obstacle_avoidance</a>  <a href="#enable_obstacle_avoidance" style="float:right;text-align:right;">obstacle_avoidance_enable</a>
</pre>
<pre>
Topics
/<a href="./topics/camera_obstacles.html">camera_obstacles</a>
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
<a href="#obstacle_avoidance_enable">obstacle_avoidance_enable(request, response)</a>
<a href="#camera_perception">camera_perception()</a>
<a href="#camera_recording">camera_recording()</a>

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

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA RECORDING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


<H3>camera_recording() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="camera_recording"></a>

This function is running in a thread
It creates an svo file with the date as a name and stores it at "zed_datasets/recording/".
It logs warnings if they appear and stops recording when <a href=#self.recording>self.recording</a> variable is set to False.


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA RECORDING CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


<H3>camera_recording_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="camera_recording_callback"></a>

This function manages start and stop of camera recording thread

-params:
  - request: (bool), true to start recording, false to stop recording

It logs changes

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA PERCEPTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>camera_perception()<a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="Obstacle_Avoidance"></a>

it selects GPU for calculation, loads the Weights and uses YoloV5.

It reads the image sent by camera, preprocesses it and detects objects, afterwards it returns the objects detected (stored in <a href=#self.detections>self.detections</a>) to the camera.

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% IMG PREPROCESS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>img_preprocess(img,device,half,net_size)<a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="img_preprocess"></a>

This function preprocesses the image as specified in [zed examples](https://github.com/stereolabs/zed-examples/blob/master/object%20detection/custom%20detector/python/pytorch_yolov5/detector.py)

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% XYWH TO ABCD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>xywh2abcd(xywh,im_shape)<a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="Obstacle_Avoidance"></a>

This function calculates bounding box of the object detected as specified in [zed examples](https://github.com/stereolabs/zed-examples/blob/master/object%20detection/custom%20detector/python/pytorch_yolov5/detector.py)

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DETECTIONS TO CUSTOM BOX %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>detections_to_custom_box(detections,im,im0)<a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="Obstacle_Avoidance"></a>

This function makes use of the object detections to calculate bounding boxes, it uses <a>xywh2abcd(xywh,im_shape)</a>

for more information check [zed examples](https://github.com/stereolabs/zed-examples/blob/master/object%20detection/custom%20detector/python/pytorch_yolov5/detector.py)

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA RECORDING CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->


<H3>main() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/camera_node.py#L62" style="float:right;text-align:right;">code</a></H3>
<a id="camera_recording_callback"></a>

The main loop is in charge of creating the messages sent to the ASV.

After obtaining the objects and their positions it translates them to a 108º read. from -54 to 54º

It will read all the objects detected and their distance,

using pythagoras it will calculate their angles, if there are 2 objects occupying the same angles the closer one will take preference.


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->