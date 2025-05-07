Install below dynamixel SDK

```
pip install dynamixel-sdk

```

To control the pan and tilt motor motions execute below command with default position values

- pan motor position limits : [90, 220] in degrees
- tilt motor position limits : [100, 250] in degrees

```
roslaunch steve_pan_tilt_controller steve_pan_tilt_controller.launch
```

To control using explicit goal positions

```
roslaunch steve_pan_tilt_controller steve_pan_tilt_controller.launch pan_goal_position:=90 tilt_goal_position:=180
```
