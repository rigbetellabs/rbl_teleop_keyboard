# rbl_teleop_keyboard
Generic Keyboard Teleop for ROS

# Launch
Run.
```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py
```

With custom values.
```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py _speed:=0.9 _turn:=0.8
```

Publishing to a different topic (in this case `my_cmd_vel`).
```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py cmd_vel:=my_cmd_vel
```

# Usage
```
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

1 : store pose 1    ! : navigate to stored pose 1
2 : store pose 2    @ : navigate to stored pose 2
3 : store pose 3    # : navigate to stored pose 3

H : Return Home
Tab : clear costmaps
Enter : cancel current goal

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

# Repeat Rate

If your mobile base requires constant updates on the cmd\_vel topic, teleop\_twist\_keyboard can be configured to repeat the last command at a fixed interval, using the `repeat_rate` private parameter.

For example, to repeat the last command at 10Hz:

```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py _repeat_rate:=10.0
```

It is _highly_ recommened that the repeat rate be used in conjunction with the key timeout, to prevent runaway robots.

# Key Timeout

Teleop\_twist\_keyboard can be configured to stop your robot if it does not receive any key presses in a configured time period, using the `key_timeout` private parameter.

For example, to stop your robot if a keypress has not been received in 0.6 seconds:
```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py _key_timeout:=0.6
```

It is recommended that you set `key_timeout` higher than the initial key repeat delay on your system (This delay is 0.5 seconds by default on Ubuntu, but can be adjusted).

# Twist with header
Publishing a `TwistStamped` message instead of `Twist` can be enabled with the `stamped` private parameter. Additionally the `frame_id` of the `TwistStamped` message can be set with the `frame_id` private parameter.
```
rosrun rbl_teleop_keyboard rbl_teleop_keyboard.py _stamped:=True _frame_id:=base_link
```