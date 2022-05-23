---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
title: Behaviour
---



The ASV is a vehicle that sails over the water equiped with few sensors to measure the quality of the water and send them to a server.

<FONT COLOR="#999900"> Start phase:<br>
</FONT>

1. The drone starts in a safe mode disarmed, initializing connection on the server, logging status.
2. On server, Indicate one or several samplepoints to collect clicking on the map, or select a preloaded mission (created with [google earth](https://earth.google.com/web/) and stored as KML)
3. The drone will change mission mode to:
  - "2" Preloaded mission
  - "4" Simple point Mission
4. After confirming that the drone is in a good status (reporting problems if they arise and rejecting mission), it will move towards the mission points
5. Distance towards the point as well as position of the drone and mission point are reported and showed in an image at server.
6. Once point is reached drone will stop, maintaining position if currents are present, take the number of samples specified and send them to server real time.
7. If there are points left it will return to step 4.
8. If the drone has finished mission it will update status and wait for instructions

![main page](./assets/main_page.png)


<FONT COLOR="#999900"> Extra behaviours:<br>
</FONT>

- For safety You are able to stop the code reaching a manual state of the drone by pulling the lever of the RC
- If the drone is in a wrong status (no GPS, IMU problems, etc) it wont start a mission and it will be reported to the server, always check the EKF status at server main page.
- The drone has extra functions like, use a planner, avoid obstacles. these are reported in the corresponding parts of the code
- The server has a tab to manage parameters, check [config](./src/asv_loyola_us/config/config.html) for a complete param list

![params tab](./assets/params_tab.png)

this project had 2 original statements of how it should work

- [first statement](https://docs.google.com/document/d/1aB7zIHbFlCjMJU5NLA3QePaBbxN2uUrcRRFxqOyA-6Y/edit?usp=sharing)
  - [diagram](https://drive.google.com/file/d/1wTaNIGYpCW7MTvH8JHc5deldHkSNIUf7/view?usp=sharing)

- [second statement](https://docs.google.com/document/d/1gskgmtnL9DoJ_OiX-3LTHiPfkZdP9JaCFyxJj1FoR2w/edit?usp=sharing)
  - [diagram](https://drive.google.com/file/d/1ER4REKppzclUCrOkx840AATA3piudeuC/view?usp=sharing)

But it has evolved until reaching this solution:

![diagram](./assets/UML%20Diagram.jpg)

To know more about the code start with [First Steps in understanding the code](./src/Xavier.html)