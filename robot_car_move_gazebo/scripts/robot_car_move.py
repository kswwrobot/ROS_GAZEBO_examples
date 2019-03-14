#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int32MultiArray

if __name__ == "__main__":    
    
    # 建立成ROS的一個node
    rospy.init_node('move_robot_car', anonymous=True)
    
    # publish輪子的速度到/robot_car/vel_cmd plugin的subscriber會去接收這個速度的值，並設定輪子的速度    
    pub = rospy.Publisher("/robot_car/vel_cmd", Int32MultiArray, queue_size=10)          
    rate = rospy.Rate(60) # 60hz
    
    # 輪子速度的格式可以用Int32MultiArray，因為在plugin robot_car_set_wheel_joint_velocity.cc也是這樣接收的，publisher跟subsriber必須有相同的msg格式
    wheel_vels = Int32MultiArray()
    wheel_vels.data = [10, 10]
    
    while not rospy.is_shutdown():            

        try:
            pub.publish(wheel_vels)
            rate.sleep()                
            
        except Exception as e:
            break
            
