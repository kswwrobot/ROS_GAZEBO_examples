#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import threading
from nn import Net
import torch
import torch.nn as nn
import cma
import gym
import gym_robobo_predator_prey
from nn import Net

def tensors_to_params_list(tensors):
    
    params_list = []
    
    for tensor in tensors:
        print('-----------')
        print(tensor.shape)
        if len(tensor.shape) == 1:
            params_list += tensor.tolist()

        elif len(tensor.shape) == 2:
            l = tensor.tolist()
            flat_list = [item for sublist in l for item in sublist]
            params_list += flat_list
        else:
            raise Exception("Wrong dim")
            
    return params_list
    
def params_list_to_state_dict(params_list, net):

    new_state_dict = net.state_dict()
    start_idx = 0
    end_idx = 0
    for state, params in net.state_dict().items():        
        end_idx = start_idx + params.numel()
        
        new_state_dict[state] = torch.nn.Parameter(torch.tensor(params_list[start_idx:end_idx]).view(params.shape))
        
        start_idx = end_idx
    
    return new_state_dict  

event_episode_start = threading.Event()
event_episode_end = threading.Event()
count_episode_start = 0

event_episode_start.daemon = True
event_episode_end.daemon = True

num_predators = 3
nets = []
for i in range(num_predators):
    nets += [Net(8, 2)]

def cma_train(net, obj_function):
    params_list = tensors_to_params_list(list(net.parameters()))

    es = cma.CMAEvolutionStrategy(params_list, 0.5)    
        
    es.optimize(obj_function)

def set_event_episode_start():
    
    global num_predators, count_episode_start, event_episode_start
    
    count_episode_start += 1
    if count_episode_start == (num_predators + 1):
        print("Episode can start")
        event_episode_end.clear()
        count_episode_start = 0
        event_episode_start.set()
        
    print("episode_start.wait")  
    event_episode_start.wait()
    print("episode_start.wait_finish")
    
def predator_obj_function_helper(params_list, nets, idx):

    global count_episode_start, event_episode_start, event_episode_end

    new_state_dict = params_list_to_state_dict(params_list, nets[idx])
    
    #if idx == 2:
        #print(new_state_dict)
    #    print("XDDD")
    
    nets[idx].load_state_dict(new_state_dict)    
    
    set_event_episode_start()
    
    print("episode_end.wait")    
    event_episode_end.wait()
    print("episode_end.wait_finish")    
    print("set fitness")
    
    fitness = np.random.rand()
    
    return fitness

def predator1_obj_function(params_list):    

    global nets
    
    fitness = predator_obj_function_helper(params_list, nets, 0)
    
    print("return fitness")
    
    return fitness
    
def predator2_obj_function(params_list):

    global nets
    
    fitness = predator_obj_function_helper(params_list, nets, 1)
    
    print("return fitness")
    
    return fitness
    
def predator3_obj_function(params_list):    

    global nets
    
    fitness = predator_obj_function_helper(params_list, nets, 2)
    
    print("return fitness")
    
    return fitness


t = threading.Thread(target = cma_train, args = (nets[0], predator1_obj_function))
t.daemon = True
t.start()

t = threading.Thread(target = cma_train, args = (nets[1], predator2_obj_function))
t.daemon = True
t.start()

t = threading.Thread(target = cma_train, args = (nets[2], predator3_obj_function))
t.daemon = True
t.start()

env = gym.make('gym_robobo_predator_prey-v0')

done = False
count_episode = 0

while not rospy.is_shutdown():
            
    count_episode += 1
    print('episode:', count_episode)
    
    observations = env.reset()
    done = False        
    
    set_event_episode_start()        
    
    while not done and not rospy.is_shutdown():
        action = np.zeros((num_predators, 2), dtype=int)
        for predator_idx, obs in enumerate(observations):
            if obs == None:
                continue
            obs_img = obs[-1]            
            obs = obs[0:8]
            if type(obs_img) != type(None):
                window_name = "predator" + str(predator_idx + 1) + "_image"
                cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
                cv2.moveWindow(window_name, 0, 30 + predator_idx * 350)
                cv2.imshow(window_name, obs_img)
                cv2.resizeWindow(window_name, 300,300)
                cv2.waitKey(1)            
            action[predator_idx,:] = nets[predator_idx](torch.FloatTensor(obs)).numpy()
        observations, reward, done, _ = env.step(action)        
               
    print("Clean start event")
    event_episode_start.clear()    
        
    print("Save fitness info")
    print("Episode end")
    event_episode_end.set()
        
    
    
