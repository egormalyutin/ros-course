#!/bin/bash

cmd_vel() {
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "
    linear: 
        x: $1
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: $2
    " ${@:3} > /dev/null
}

cmd_vel 1.0  0.785398163 -r 10 -t 71
cmd_vel 1.0 -0.785398163 -r 10 -t 71