---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Planner Node</H1> 

This node is in charge of calculating a path for the UAV to move


<FONT COLOR="#ff0000"> TODO:<br>
- Right now is indevelopment as we need to figure out a bit map of obstacles and how to include the camera for obstacle detection<br>
- We need a get map function</FONT>

<pre>
Services
/
</pre>
<pre>
Topics
/
</pre>
<pre>
Actions
/<a href="./actions/go_to.html">go_to</a>  <a href="#go_to_callback" style="float:right;text-align:right;">go_to_callback</a>
</pre>

<pre>
functions
<a href="#get_path">get_path(goal)</a>
<a href="#go_to_callback">go_to_callback(goal_handle)</a>
</pre>


<pre>
variables
<a id="self.path">path</a>
<a id="self.map">map</a>
<a id="self.status">status</a>
</pre>

<pre>
Parameters
<a href="./parameters/debug.html">debug</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET PATH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>get_path(goal) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/planner_node.py#L91" style="float:right;text-align:right;">code</a></H3>
<a id="get_path"></a>

This function calculates the path to go from the actual position to the goal point

-params:
  - goal: reference point


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GO TO CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>go_to_callback(goal_handle) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/1265f7548ce48155cd95fefedaae14bf958d1361/src/asv_loyola_us/asv_loyola_us/planner_node.py#L52" style="float:right;text-align:right;">code</a></H3>
<a id="go_to_callback"></a>

This function executes the action in charge of going to the goal point

- goal_handle
  - request:
    - samplepoint: reference point
  - feedback:
    - 
  - resutl:
    - success: True if point reached false otherwise

<FONT COLOR="#ff0000"> TODO:<br>
- In development </FONT>



<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->