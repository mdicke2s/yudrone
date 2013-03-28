yudrone
=======

Ardrone 2.0 Navigation with ROS:
* depends on ardrone_autonomy (https://github.com/AutonomyLab/ardrone_autonomy)
* developed within a computer science project at York University, Toronto (https://www.cse.yorku.ca/cshome/)
* Vision, Graphics, and Robotics Laboratory (http://vgrserver.cse.yorku.ca)

Status:
* in development

Objectives:
* autonomous navigation
* Point-to-point based tasks
* augmented reality based tasks

Nodes:
* flight.py ("yudrone_flight", GUI and command line interface)
* joypad_ctrl.py ("yudrone_joy", control ardrone with ps3-like joypad)
** left joystick (horizontal movements)
** right joystick (altitude and yaw)
** buttons (takeoff, land, emergency, stop_twist)
* commands.py ("yudrone_cmd", offers different naviagtion commands)
* setDefaultParameter.py ("yudrone_param", sets environment using rosparam)

Topics:
* "yudrone/commands", commandsMsg, yudrone_cmd listenes command messages
* "yudrone/lock_cmd", Int32, yudrone_cmd reports lock (if -1 it is blocked for incomming commands)
* "yudrone/lock_joypad", Bool, is used to lock joysticks during task performance

To install yudrone:
* make all dependent packages
* copy add_to_ar_recog_bin.zip into .../ar_recog/bin
* modify setDefaultParameter.py

For more information on the software read file and function headers.

TODOs before field trials
* demo videos
* poster
* boxes
* paper draft
* thesis draft
* (project report for HBRS)

Hints:
```sh
#ardrone_autonomy cli
rosrun ardrone_autonomy ardrone_driver [-ip x.x.x.x]

#git cli
git add * [or <file>]
git commit -m "message"
git push origin master
git pull
```