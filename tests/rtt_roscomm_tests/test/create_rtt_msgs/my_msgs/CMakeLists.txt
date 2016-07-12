cmake_minimum_required(VERSION 2.8.3)
project(my_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs message_generation)

add_message_files(FILES m1.msg m2.msg m3.msg m4.msg)
add_service_files(FILES s1.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)


