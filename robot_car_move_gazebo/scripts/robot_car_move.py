#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from std_msgs.msg import Int32MultiArray
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-n", "--robot_namespace", help="Namespace of the robot", dest="ns", default="/robot_car")
args = parser.parse_args()
NS = args.ns

if __name__ == "__main__":    
    
    rospy.init_node('move_robot_car', anonymous=True)
    
    pub = rospy.Publisher("/robot_car" + "/vel_cmd", Int32MultiArray, queue_size=10)          
    rate = rospy.Rate(10) # 10hz
    wheel_vels = Int32MultiArray()
    wheel_vels.data = [10, 10]
    
    while not rospy.is_shutdown():            

        try:
            pub.publish(wheel_vels)
            rate.sleep()                
            
        except Exception as e:
            break
            
