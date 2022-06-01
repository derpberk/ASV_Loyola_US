---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page

---
<H1>Planner Node</H1> 

This node is in charge of calculating a path for the ASV to move


<FONT COLOR="#ff0000"> TODO:<br>
- Use maps with cost instead of binnary, ponder other planners different from A_star</FONT>

<pre>
Services
/<a href="./services/load_map.html">load_map</a>  <a href="#load_map" style="float:right;text-align:right;">load_map_callback</a>
/<a href="./services/enable_planning.html">enable_planning</a>  <a href="#enable_planning" style="float:right;text-align:right;">enable_planning_callback</a>
/<a href="./services/cancel_movement.html">calculate_path</a>  <a href="#calculate_path" style="float:right;text-align:right;">calculate_path_callback</a>
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



<a href="#load_map_callback">load_map_callback(request, response)</a>
<a href="#enable_planning_callback">enable_planning_callback(request, response)</a>
<a href="#calculate_path_callback">calculate_path_callback(request, response)</a>
</pre>


<pre>
variables
<a id="self.use_planner">use_planner</a>
<a id="self.planner">planner</a>
<a id="self.status">status</a>
</pre>

<pre>
Parameters
<a href="./parameters/debug.html">debug</a>
<a href="./parameters/map_filename.html">map_filename</a>
</pre>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ENABLE PLANNER CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>enable_planning_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/planner_node.py#L80" style="float:right;text-align:right;">code</a></H3>
<a id="enable_planning_callback"></a>

This function enables or disables planning

- request:
  - value: (Boolean) True to enable, False to disable planning
- response:
  - success: always True


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CALCULATE PATH CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>calculate_path_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/planner_node.py#L91" style="float:right;text-align:right;">code</a></H3>
<a id="calculate_path_callback"></a>

This function calculates the path towards a point if planner is enabled, returns destination if planner is not enabled (logs time spent calculating path)

- request:
  - new_point: (Location) Point where we want to take a samplepoint
- response:
  - point_list: (Location[]) Tuple containing all the waypoints to reach destination
  - success: (Bool) True upon path, false otherwise


Before calculating the path it takes into account the following things

- If drone is Offline returns False

- If we use planner
  - If we are outside the map return False
  - If drone or destination is inside a wall return False

If there is no path return false
Otherwise return path and True

The planner is well commented in <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/submodulos/A_star.py" style="float:right;text-align:right;">A_Star</a></H3>

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOAD MAP CALLBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>load_map_callback(request, response) <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us/asv_loyola_us/planner_node.py#L67" style="float:right;text-align:right;">code</a></H3>
<a id="load_map_callback"></a>

This function loads a map inside the planner if map exists, returns response.success = False if load fails 

- request:
  - file_name: (str) string containing the name of the map
- response:
  - success: (Bool) True upon success

You can check available maps in [maps section](../../../maps.html)


<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->