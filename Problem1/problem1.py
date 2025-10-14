import numpy as np
import mujoco
import time
import math
from mujoco import viewer
from .. import dmmotor
from dmmotor import dmmotor

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
        flag = False #flag to indiciate if calculations have been done during hold
        friction_torque = 0
        applied_torque = 0
        shaft_weight = 0.1 #change later
        while(time.time() - cur_time < 0.001):
            if(flag == False):
                flag = True 
                motor.friction_torque(mj_data.qvel[0], mj_data.qvel[0], applied_torque) #change parameters later
                #complete calculations during hold
            x = 1 + 1 #dummy operations
        mujoco.mj_step(mj_model, mj_data)
        cur_time = time.time()
        viewer_window.sync()
main()




