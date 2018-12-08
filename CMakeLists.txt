cmake_minimum_required(VERSION 2.8.3)
project(ros_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs visualization_msgs tf tf_conversions message_generation)

# add message files:
add_message_files(
   FILES
   JointCommand.msg
   )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
   CATKIN_DEPENDS rospy roscpp std_msgs visualization_msgs tf tf_conversions message_runtime
   )

include_directories(include
  ${catkin_INCLUDE_DIRS}
)



