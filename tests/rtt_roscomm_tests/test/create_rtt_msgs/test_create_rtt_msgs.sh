#!/bin/sh -e

# Create a temporary catkin workspace
TEMP_WS=`mktemp -d`
mkdir -p $TEMP_WS/src
cp -R `dirname $0`/my_msgs $TEMP_WS/src/my_msgs
rm $TEMP_WS/src/my_msgs/CATKIN_IGNORE

# Build the message package
mkdir -p $TEMP_WS/build/my_msgs
cd $TEMP_WS/build/my_msgs
cmake $TEMP_WS/src/my_msgs -DCATKIN_DEVEL_PREFIX=$TEMP_WS/devel -DCMAKE_INSTALL_PREFIX=$TEMP_WS/install
make install

# Generate and build the rtt typekit package
. $TEMP_WS/install/setup.sh
cd $TEMP_WS/src
rosrun rtt_roscomm create_rtt_msgs my_msgs
mkdir -p $TEMP_WS/build/rtt_my_msgs
cd $TEMP_WS/build/rtt_my_msgs
cmake $TEMP_WS/src/rtt_my_msgs -DCATKIN_DEVEL_PREFIX=$TEMP_WS/devel -DCMAKE_INSTALL_PREFIX=$TEMP_WS/install
make install

# Run RTT and try to import the typekit
cd $TEMP_WS
rosrun rtt_ros rttscript --check -l debug `dirname $0`/test_create_rtt_msgs.xml

# Cleanup
rm -rf $TEMP_WS
