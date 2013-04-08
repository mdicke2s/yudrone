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

TODOs:
* 1 demo videos   (due Apr17)
* 2 poster        (due Apr17)
* 3 boxes         (due Apr10)
* 4 final report  (due Apr25)
* 5 thesis draft  (due Apr25)
* 6 (paper draft)

Schedule:
<table>
  <tr>
    <th>Day</th>
    <th>Time</th>
    <th>goal</th>
  </tr>
  <tr>
    <td>Fr 5</td>
    <td>8am-3pm</td>
    <td>state machine and yudrone_cmd are joined</td>
  </tr>
  <tr>
    <td>Weekend</td>
    <td>???</td>
    <td>in yudrone_cmd: timers are substituted with event-queue</td>
  </tr>
  <tr>
    <td>Mo 8</td>
    <td>10am-4pm</td>
    <td></td>
  </tr>
  <tr>
    <td>Tu 9 or We 10</td>
    <td>8am-6pm</td>
    <td>experimental run successfull</td>
  </tr>
  <tr>
    <td>Th 11</td>
    <td>3am-6pm</td>
    <td>box(es) build and tags printed</td>
  </tr>
  <tr>
    <td>Fr 12</td>
    <td>8am-5pm</td>
    <td>video</td>
  </tr>
  <tr>
    <td>Weekend</td>
    <td>poster</td>
    <td></td>
  </tr>
  <tr>
    <td>Mo 15</td>
    <td>8am-5pm</td>
    <td></td>
  </tr>
  <tr>
    <td>Tu 16</td>
    <td>8am-5pm</td>
    <td>poster printed</td>
  </tr>
</table>

Notes:
```sh
#ardrone_autonomy cli
rosrun ardrone_autonomy ardrone_driver [-ip x.x.x.x]

#git cli
git add * [or <file>]
git commit -m "message"
git push origin master
git pull
```
