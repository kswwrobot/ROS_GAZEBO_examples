cmake_minimum_required(VERSION 2.8.3)                   #定義catkin用的CMAKE版本
project(robot_car_move_gazebo)                          #project名稱，編譯後ros會記住這個名字

#找到需要的package
find_package(catkin REQUIRED COMPONENTS                 
    roscpp
    rospy
    std_msgs
    gazebo
)

include_directories(${catkin_INCLUDE_DIRS})             #加入編譯需要的路徑，讓系統找到的重要的檔案。

#當有其他package依賴，這個robot_car_move_gazebo package時，要告訴別人要有哪些關於catkin的dependencies
catkin_package(CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs    
)                                        

include_directories(${GAZEBO_INCLUDE_DIRS})             
link_directories(${GAZEBO_LIBRARY_DIRS})                #Library的路徑
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")      #編譯的FLAG

#install(DIRECTORY launch
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})    

#install(DIRECTORY worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(robot_car_set_wheel_joint_velocity SHARED plugin/robot_car_set_wheel_joint_velocity.cc)        #把Plugin加入library
target_link_libraries(robot_car_set_wheel_joint_velocity ${GAZEBO_LIBRARIES})                              
