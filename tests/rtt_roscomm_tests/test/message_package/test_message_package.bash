#!/usr/bin/env bash

# Get the script path
pushd $(dirname $0) > /dev/null
SCRIPTPATH=$(pwd -P)
popd > /dev/null

# Create a temporary catkin workspace
TEMP_WS=$(mktemp -d)
mkdir $TEMP_WS/src
cp -R $SCRIPTPATH/my_msgs $TEMP_WS/src/my_msgs
rm $TEMP_WS/src/my_msgs/CATKIN_IGNORE

# Build the message package
pushd $TEMP_WS
catkin_make -j1

# Generate and build the rtt typekit package
source devel/setup.bash
cd src
rosrun rtt_roscomm create_rtt_pkg my_msgs
cd ..
catkin_make -j1
