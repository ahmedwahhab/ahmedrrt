cmake_minimum_required(VERSION 2.8.3)
project(ahmedrrt)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  tf
  visualization_msgs
)



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Num.msg
   #Message2.msg
 )


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AddTwoInts.srv
   makePlan.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  
)

catkin_package( 

CATKIN_DEPENDS message_runtime
)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


