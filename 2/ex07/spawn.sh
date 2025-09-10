#!/bin/bash

ros2 service call /reset std_srvs/srv/Empty
ros2 param set /turtlesim background_g 124

spawn() {
    ros2 service call /spawn turtlesim/srv/Spawn "
        x: $1
        y: $2
        theta: 0.0
        name: '$3'
    "
}

spawn 3 3 Leonardo
spawn 8 3 Raphael
spawn 3 8 Donatello
spawn 8 8 Michelangelo