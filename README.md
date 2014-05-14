yudrone
=======

Ardrone 2.0 Navigation with ROS:
* depends on ardrone_autonomy (https://github.com/AutonomyLab/ardrone_autonomy)
* developed within a computer science project at York University, Toronto (https://www.cse.yorku.ca/cshome/)
* Vision, Graphics, and Robotics Laboratory (http://vgrserver.cse.yorku.ca)
* Demo-Video and poster (http://www.andrewspeers.com/Andrew_Speers/ARDrone_Graph_Traversal.html)

Status:
* in development

Objectives:
* autonomous navigation
* Point-to-point based tasks
* augmented reality based tasks

Nodes:
* flight.py ("yudrone_flight", GUI and command line interface)
* joypad_ctrl.py ("yudrone_joy", control ardrone with ps3-like joypad)
  * left joystick (horizontal movements)
  * right joystick (altitude and yaw)
  * buttons (takeoff, land, emergency, stop_twist)
* commands.py ("yudrone_cmd", offers different naviagtion commands)
* setDefaultParameter.py ("yudrone_param", sets environment using rosparam)

Topics:
* "yudrone/commands", commandsMsg, yudrone_cmd listenes command messages
* "yudrone/cmdStatus", commandStatus,
  * yudrone_cmd reports availability for incomming commands (lock = false)
  * yudrone_cmd reports block for incomming commands (lock = true, id = command_that_is_executed)
  * yudrone_cmd reports status changes for commands beeing procecced (status = running/done_successfully/errorXYZ, id = command_that_is_executed)
* "yudrone/lock_joypad", Bool, is used to lock joysticks during task performance

To install yudrone:
* make all dependent packages
* modify setDefaultParameter.py
* copy add_to_ar_recog_bin.zip into .../ar_recog/bin
* (print tags from same zip file)

For more information on the software read file and function headers.

Notes:
TARGET_APPROACH target is supposed to work like this
* face and wait for sum of errors to be low (stable position at 1600mm distance)
* gently decrease distance
 * best case until tag fills video frame
 * use PID controller on ax (yaw)
* go on top
 * swicht to bottom camera
 * elevate until tag in range
 * hover to certain altitude using controller on lx and ly
If we are on top of a target we can do a search which is cleverer than the current one, but we can also hover on top and use predefined graph to determine the next tags position.

Helpful commands:
```sh
#ardrone_autonomy cli
rosrun ardrone_autonomy ardrone_driver [-ip x.x.x.x]

#git cli
git add * [or <file>]
git commit -m "message"
git push origin master
git pull
```

