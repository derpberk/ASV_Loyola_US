/ardupilot/ardurover -S -I0 --home 37.4186882767512,-6.001257728881311,200,500 --model Rover --defaults /ardupilot/copter.parm &
mavproxy.py --master=tcp:127.0.0.1:5760 --out=tcpin:0.0.0.0:5798 --out=tcpin:0.0.0.0:5788

