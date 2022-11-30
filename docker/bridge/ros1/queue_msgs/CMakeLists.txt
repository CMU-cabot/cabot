cmake_minimum_required(VERSION 2.8.3)
project(queue_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation people_msgs std_msgs)

add_message_files(
   DIRECTORY msg
   FILES
    Queue.msg
)

generate_messages(
   DEPENDENCIES   geometry_msgs people_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime people_msgs std_msgs
)
