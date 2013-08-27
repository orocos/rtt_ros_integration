cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_srvs)

find_package(catkin REQUIRED COMPONENTS genmsg rtt_roscomm std_srvs)

catkin_package(CATKIN_DEPENDS genmsg rtt_roscomm std_srvs)

ros_generate_rtt_service_proxies(std_srvs)

orocos_generate_package()
