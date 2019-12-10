# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(turtle_package)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)#std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

#add_dependencies(talker talker_listener_generate_messages_cpp)

# %EndTag(FULLTEXT)%

