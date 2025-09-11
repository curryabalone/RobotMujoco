import pinocchio as pin
import numpy as np
import mujoco
import time
import math
from mujoco import viewer

def calculate_gravity(motorangle):
    return abs(math.cos(motorangle)*9.81*(0.150*0.15 + 0.362*0.3))


def main():
    computermotor = True
    mjcf_path = "./RobotMujoco.xml"
    mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
    mj_data = mujoco.MjData(mj_model)
    viewer_window = viewer.launch_passive(mj_model, mj_data)
    print(mj_model.opt.timestep)
    cur_time = 0
    motordelta = 0.0
    motorangle = -3.14/2
    while True:
        tmp = mj_data.qpos[0]
        motordelta = tmp - motorangle
        motorangle = tmp
        motorspeed = motordelta/0.001
        exernal_torque = calculate_gravity(motorangle)
        #let's try to write a simple dampening program!
        mj_data.ctrl[0] = -1*motorspeed*0.1*0.1
        while(time.time() - cur_time < 0.001):
            x = 1 + 1
        mujoco.mj_step(mj_model, mj_data)
        cur_time = time.time()
        viewer_window.sync()
main()




