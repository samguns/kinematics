#! /bin/bash
x-terminal-emulator -e roslaunch kuka_arm target_description.launch &
sleep 3 &&
x-terminal-emulator -e roslaunch kuka_arm cafe.launch
