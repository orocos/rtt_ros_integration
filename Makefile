default: patched-ros all

include $(shell rospack find mk)/cmake.mk

PATCH=`rospack find rtt_ros_integration`/boost-serialization.patch

patched-ros:
	sudo -E patch -p0 -d `rosstack find ros` -i $(PATCH)
	touch patched-ros
wipe: clean
	sudo -E patch -p0 -R -d `rosstack find ros` -i $(PATCH)
	rm patched-ros