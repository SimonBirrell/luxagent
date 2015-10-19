#!/bin/bash

# An alternative launch point that re-executes luxagent.py if
# for any reason it exits (i.e. if it crashes)
#
# roslaunch luxagent luxagent.sh
#

while :
do
	echo "Press [CTRL+C] to stop.."
	sleep 1
	rosrun luxagent luxagent.py
done