# RBE502-BalanceBot
For the project 4 differnt types of controllers were used with one controller being used in both statespace implementation and basic impementation
The differnt controllers are:<br>
PID<br>
PD<br>
PD feed forward<br>
PD feed forward in statespace<br>
compute torque in statespace<br>

## Launch options:
controller := <br>
  default = 1<br>
  0 = no controller<br>
  1 = PID controller<br>
  2 = PD controller<br>
  3 = PD controller with feed forward<br>
  4 = PD controller with feed forward in State Space<br>
  5 = Computed Torque in State Space<br>
 
use_rqt := <br>
  default = false<br>
  display rqt plots <br>

record_bag := <br>
  default = true<br>
  record a rosbag for this run

numSpeedControllers := <br>
  default = 3<br>
  Option for the amount of controller that use speed control. Should not be changed as no new controllers are being created <br>

  
Open terrain world: 

```
roslaunch teeterbot_gazebo teeterbot_terrain_world.launch
```
Open keyboard controller: 
```
roslaunch teeterbot_gazebo control.launch
```

