---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
<H1>Handler Node</H1> 

This node is in charge of the handshake between the ASV and the server


<FONT COLOR="#ff0000"> TODO:<br>
- Optimise code<br>
- translate messages</FONT>

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
/
</pre>

<pre>
functions
<a href="#check_connections">check_connections()</a>
</pre>


<pre>
variables
<a id="self.active_drones">active_drones</a>
</pre>

<pre>
Parameters
</pre>


<!-- %%%%%%%%%%%%%%%%%%%%%%%%% START OF FUNCTION DEFINITIONS AREA %%%%%%%%%%%%%%%%%%%%%%%%%% -->

<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Check connections %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -->

<H3>check_connections() <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/src/asv_loyola_us_server/asv_loyola_us_server/handler.py" style="float:right;text-align:right;">code</a></H3>
<a id="check_connections"></a>
This function checks for the active nodes. If a new ASV appears it adds it to the "active_drones" list. If all the nodes related to a drone disappear, it will trigger a message and delete the drone from the list

- managed variables
  - <a href="#self.active_drones">active_drones</a>




