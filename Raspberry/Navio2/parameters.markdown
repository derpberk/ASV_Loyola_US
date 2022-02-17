---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

The first step in order to configure a new drone is to configure the general parameters of the Navio2.

For this you must have installed [mission planner](/404.html)

once installed you must

1power on the navio
2Connect the Raspberry pi to internet
3Connect your computer to the same internet

Once connected, open mission planner, on the topright options select TCP, click connect and use as ip "navio" and as port "5678" as we have used it as broadcas when configuring the raspberry pi.

If everything has been done right, mission planner must connect (if you just restarted the Raspberry, it may spend 2 minutes to startup) if there is a problem check [solver](./parameters_solve.html)

Download the general parameter list from <a href="../../assets/ASV_parametres.param">here</a>

Once connected navigate to the tab "Config".
click on Full parameters list
on the right you must find a buttom called, load params.

While loading you may see some errors due to not configured sensors and read_only parameters. Just click yes.

Once finished ardupilot must be configured with our frame, ¡Congratulations!

Now you may follow the [Next step](./PID.html)