import mujoco
import mujoco_viewer
import numpy as np
import sys
import math
import time
import generate_worm as gw
import random


parts = gw.parts
#Load model from XML file
model = mujoco.MjModel.from_xml_path('model.xml')  #Make sure to specify the correct path

#Create MjData instance for simulation data
data = mujoco.MjData(model)

# Initialize viewer
viewer = mujoco_viewer.MujocoViewer(model, data)


def generate_random_bit():

    return random.choice([0, 1])

def gen_random(a,b):

    return random.uniform(a,b)



frequency = gen_random(0.5, 1.5)  #Oscillation frequency in Hz
amplitude = np.deg2rad(15)  #Amplitude in radians

prev_position = np.array([data.qpos[0], data.qpos[1]]) 
prev_time = data.time
fitness_scores = []

i = 0
while True:
    
    current_time = data.time
    current_position = np.array([data.qpos[0], data.qpos[1]])  

    #Calculate the current time in the simulation
    t = data.time
    
    #calculate the angle for each joint
    angle = amplitude * np.sin(2 * np.pi * frequency * t)
    angle2 = amplitude * np.cos(2 * np.pi * frequency * t)
    
    # Update the joint angles directly
    for i in range(parts):
        start = 6
        data.qpos[start+i] = angle
        # if i%2 == 0:
        #     data.qpos[start+i] = angle 
        # else:
        #  data.qpos[start+i] = angle2  


        # Print qpos values
    if i== 0:
        print(f"qpos values: {data.qpos}")
        i = 1
    
    mujoco.mj_step(model, data)
    
    # Update the viewer
    viewer.render()

    
    if current_time - prev_time >= 0.1: 
        #Calculate displacement as the difference between current and previous positions
        displacement = current_position - prev_position
        #Calculate speed as the magnitude of displacement divided by time interval
        speed = np.linalg.norm(displacement) / (current_time - prev_time)
        #Append the calculated speed to the fitness scores
        fitness_scores.append(speed)
        print(f"Current speed: {speed} units/sec")

        #Update previous position and time
        prev_position = current_position
        prev_time = current_time
    


