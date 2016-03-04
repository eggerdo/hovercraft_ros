#!/bin/bash

gnome-terminal 	--geometry=$GEOMETRY \
	--tab -e "bash -E -c -i 'roslaunch hovercraft_teleop ps3_teleop.launch'" \
	--tab -e "sixad -s"
	# ps3joy doesn't work under indigo and 14.04 as of now (31.07.2014)
	# --tab -e "sudo bash -E -c -i 'rosrun ps3joy ps3joy.py'"
