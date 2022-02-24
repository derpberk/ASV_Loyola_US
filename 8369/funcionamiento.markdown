---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
#title: Behaviour
---

The ASV behaviour is composed of x stages

1. The communication with the autopilot controlling the drone is established
2. Wait for all the nodes to initialize
3. Drone set to Standby, motors disarmed waiting for message
4. Indicate one or several samplepoints to follow
5. The drone will change mission mode to:
  - "1" Stored mission
  - "3" Communicated Mission
6. The drone will change to "GUIDED" mode to follow the waypoints
7. Once mission is finished it will return to Standby mode

<pre>
Future updates
- Node must collect samples once a samplepoint is reached
- A planificator mode such as <a href="./code/Astar.html">A star</a> or Lazy Theta
- Check <a href="./TODO.html">List of TODO</a> for more information
</pre>




this project had 2 original statements of how it should work

- [first statement](https://docs.google.com/document/d/1aB7zIHbFlCjMJU5NLA3QePaBbxN2uUrcRRFxqOyA-6Y/edit?usp=sharing)
  - [diagram](https://drive.google.com/file/d/1wTaNIGYpCW7MTvH8JHc5deldHkSNIUf7/view?usp=sharing)

- [second statement](https://docs.google.com/document/d/1gskgmtnL9DoJ_OiX-3LTHiPfkZdP9JaCFyxJj1FoR2w/edit?usp=sharing)
  - [diagram](https://drive.google.com/file/d/1ER4REKppzclUCrOkx840AATA3piudeuC/view?usp=sharing)
