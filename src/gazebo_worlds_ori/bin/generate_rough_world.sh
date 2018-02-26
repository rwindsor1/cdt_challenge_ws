#!/bin/bash

python $(rospack find gazebo_worlds_ori)/bin/generate_rough_world.py $1 $2 $3 $4 $5 $6

# example:
# rosrun  gazebo_worlds_ori generate_rough_world.sh 12 4 0.25 100 1 odom
