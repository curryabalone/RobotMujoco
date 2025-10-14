import pinocchio as pin
import numpy as np
import mujoco
import time
import math
from mujoco import viewer
from dmmotor import motor

def calculate_gravity_torque(motorangle):
    return math.cos(motorangle)*9.81*(0.142*0.15 + 0.362*0.3) 

def main():
    # Initialize Mujoco variables
    mjcf_path = "./RobotMujocop1.xml"
    mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
    mj_data = mujoco.MjData(mj_model)
    viewer_window = viewer.launch_passive(mj_model, mj_data)
    mj_model.opt.timestep = 0.001
    motor = dmmotor(0.1, 0.01, 0.05) #fill_in values
    cur_time = 0
    while True:
        tmp = mj_data.qpos[0]
        motordelta = tmp - motorangle
        motorangle = tmp
        motorspeed = motordelta/0.001
        exernal_torque = calculate_gravity_torque(motorangle)
        #let's try to write a simple dampening program!
        flag = False #flag to indiciate if calculations have been done during hold
        friction_torque = 0
        while(time.time() - cur_time < 0.001):
            if(flag == False):
                flag = True 
                motor.friction_torque(mj_data.qvel[0], 0.362*9.81, exernal_torque) #change parameters later
                #complete calculations during hold
                
            x = 1 + 1
        mujoco.mj_step(mj_model, mj_data)
        friction_torque = 0 #torque due to friction, to be calculated
        if mj_data.qvel[0] == 0.0:
            friction_torque = 
        cur_time = time.time()
        viewer_window.sync()
main()




