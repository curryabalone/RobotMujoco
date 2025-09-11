import pinocchio as pin
import numpy as np
import mujoco
from mujoco import viewer


def main():
    mjcf_path = "./RobotMujoco.xml"
    mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
    mj_data = mujoco.MjData(mj_model)
    viewer_window = viewer.launch_passive(mj_model, mj_data)
    while True:
        mujoco.mj_step(mj_model, mj_data)
        viewer_window.sync()
main()
