#!/bin/bash
tab="--tab-with-profile=roslaunch --command "
window="--window-with-profile=roslaunch --command"

cd $HOME/catkin_ws/src/mimi_common_pkg/bashes
gnome-terminal \
    $tab './roscore.bash'\
    $tab './minimal_launch.bash'\
    $tab './3dsensor_launch.bash'\
