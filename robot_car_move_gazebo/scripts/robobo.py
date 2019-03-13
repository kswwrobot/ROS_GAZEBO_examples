#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import threading
import torch

#seperate class

class Robobo:
    def __init__(self, model_name):
        self.model_name = model_name      
        self.wheel_vels = Int32MultiArray()
        self.wheel_vels.data = [0, 0]
        self.camera_img = None        
        self.sensor_state = [0.0] * 8        
        
        self.t = threading.Thread(target = self.start)
        self.t.daemon = True
        self.t.start()

    def set_vels(self, wheel_vels_data):
        self.wheel_vels.data = wheel_vels_data
    
    def raw_image_callback(self, image_msg, name):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", image_msg)
        # Image to numpy array
        if hasattr(image_msg, "data"):        
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = np.reshape(np_arr, (image_msg.height, image_msg.width, 3))
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            self.camera_img = image_np
            #cv2.imshow(self.model_name + " image", image_np)
            #cv2.waitKey(1)
            #cv2.imshow(cv2_img)

    '''def callback(image_msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", image_msg)
        # Image to numpy array
        if hasattr(image_msg, "data"):
            #print(image_msg.height)
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #np_arr = np.reshape(np_arr, (image_msg.height, image_msg.width, 3))

            # Decode to cv2 image and store
            #print(image_np.shape)
            cv2.imshow(self.model_name + " image", image_np)
            cv2.waitKey(1)
            #cv2.imshow(cv2_img)'''
            
    def get_image(self):
        return self.camera_img
    
    def ir_sensor_callback(self, ir_msg, name):
        #print(ir_msg.range, ir_msg, name)
        if name == "ir_front_1":
            self.sensor_state[0] = ir_msg.range
        elif name == "ir_front_2":
            self.sensor_state[1] = ir_msg.range
        elif name == "ir_front_3":
            self.sensor_state[2] = ir_msg.range
        elif name == "ir_front_4":
            self.sensor_state[3] = ir_msg.range
        elif name == "ir_front_5":
            self.sensor_state[4] = ir_msg.range
        elif name == "ir_back_1":
            self.sensor_state[5] = ir_msg.range
        elif name == "ir_back_2":
            self.sensor_state[6] = ir_msg.range
        elif name == "ir_back_3":
            self.sensor_state[7] = ir_msg.range

    def timer_callback(self, event):
        global pub
        pub.publish(last_data)                                   
        
    def start(self):
        
        #rospy.Subscriber("/prey/camera1/image_raw/compressed", CompressedImage, callback)
        rospy.Subscriber("/" + self.model_name + "/camera1/image_raw", Image, self.raw_image_callback, ('image_raw'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_1", Range, self.ir_sensor_callback, ('ir_front_1'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_2", Range, self.ir_sensor_callback, ('ir_front_2'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_3", Range, self.ir_sensor_callback, ('ir_front_3'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_4", Range, self.ir_sensor_callback, ('ir_front_4'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_5", Range, self.ir_sensor_callback, ('ir_front_5'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_1", Range, self.ir_sensor_callback, ('ir_back_1'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_2", Range, self.ir_sensor_callback, ('ir_back_2'), queue_size = 2)
        rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_3", Range, self.ir_sensor_callback, ('ir_back_3'), queue_size = 2)
        
        pub = rospy.Publisher("/" + self.model_name + "/vel_cmd", Int32MultiArray, queue_size=10)          
        rate = rospy.Rate(100) # 10hz
        
        while not rospy.is_shutdown():            
            #rospy.loginfo(self.wheel_vels)
            try:
                pub.publish(self.wheel_vels)
                rate.sleep()                
                
            except Exception as e:
                #rospy.logerr("Error: %s" % (e,))
                break
                
            #print(self.model_name)
            
    
    
    
    
    
