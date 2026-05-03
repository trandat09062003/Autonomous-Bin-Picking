import os
import warnings
import logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings("ignore")
logging.getLogger().setLevel(logging.CRITICAL)
logging.basicConfig(level=logging.CRITICAL)

import time
import cv2
import matplotlib.pyplot as plt
from rlbench.action_modes import ArmActionMode, ActionMode
from main import GraspController
import numpy as np

print("Initializing RLBench Environment...")
action_mode = ActionMode(ArmActionMode.ABS_JOINT_POSITION)
grasp_controller = GraspController(action_mode, static_positions=True)
descriptions, obs = grasp_controller.reset()

print("RLBench Environment successfully launched!")
print(f"Task descriptions: {descriptions}")

objs = grasp_controller.get_objects()
home_pose = np.copy(objs['waypoint0'][1])
home_pose[0] -= 0.022
print("Moving robot to home pose...")
path = grasp_controller.get_path(home_pose)
obs, reward, terminate = grasp_controller.execute_path(path, open_gripper=True)

rgb = np.array(obs.wrist_rgb * 255, dtype='uint8')
wrist_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
cv2.imwrite('wrist_view.png', wrist_image)
print("Saved wrist camera view to wrist_view.png.")

import os
time.sleep(2)
grasp_controller.env.shutdown()
print("Test run completed successfully!")
os._exit(0)
