#!/bin/bash

cd /opt/projects/robomagellan/ros/robomagellan/nodes

chmod a+x sensorPack.py \
		  gpsLocation.py \
		  acceleration.py \
		  motors.py \
		  control.py \
		  waypoints.py \
		  navigation.py

cd /opt/projects/robomagellan/ros/robomagellan/test

chmod a+x collision-mock.py

exit 0
