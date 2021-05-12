"""
this is a python code for dynamic window approach.

model used here is bicycle model
"""
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../Support functions")

import Vehicle_model
import Node_3D

class dynamic_window_approach:
    def __init__(self,start:Node_3D,goal:Node_3D,map:Map,config:Config):
        self.start=start
        self.goal=goal
        self.map=map
        self.config=config

    def run(self):
        #state is initial state, should equal to self.start
        state=[self.start.]
        dw=self.get_dynamic_window(state)
        u,trajectory = self.get_trajectory(state,dw)
        

    

    
    def get_trajectory(self):


    def get_dynamic_window(self,state:list)->list:
        # state = [x,y,heading angle, velocity, steering angle]
        #use bicycle model, dynamic window should be [velocity, steering angle]
        Vs = [self.config.min_speed,self.config.max_speed,
         -self.config.max_steering_angle,self.config.max_steering_angle]
        
        Vd = [state[3]-self.config.max_accel * self.config.dt,
        state[3]+self.config.max_accel*self.config.dt,
        state[4]-self.config.max_steering_angle_rate*self.config.dt,
        state[4]+self.config.max_steering_angle_rate*self.config.dt]

        dw=[max(Vs[0],Vd[0]),min(Vs[1],Vd[1]),max(Vs[2],Vd[2]),min(Vs[3],Vd[3])]

        return dw
        
    def calculate_total_cost(self):

    def plot_path(self):



