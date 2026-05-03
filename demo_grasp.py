import os
import warnings
import logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings("ignore")
logging.getLogger().setLevel(logging.CRITICAL)
logging.basicConfig(level=logging.CRITICAL)

import time
import cv2
from rlbench.action_modes import ArmActionMode, ActionMode
from main import GraspController
import numpy as np
from quaternion import from_rotation_matrix, quaternion, as_rotation_matrix

print("Initializing RLBench Environment...")
action_mode = ActionMode(ArmActionMode.ABS_JOINT_POSITION)
grasp_controller = GraspController(action_mode, static_positions=True)
descriptions, obs = grasp_controller.reset()
print("RLBench Environment successfully launched!")

# Save initial scene image using left shoulder camera
rgb = np.array(obs.left_shoulder_rgb * 255, dtype='uint8')
scene_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
cv2.imwrite('scene_before.png', scene_image)
print("Saved scene_before.png")

objs = grasp_controller.get_objects()
# The EmptyContainer task usually spawns shapes. Let's find one.
# Waypoints exist for the containers. waypoint0 is large container, waypoint3 is small red container.
# We will just grab one of the shape objects.
target_obj_name = None
for name in objs.keys():
    if 'Shape' in name:
        target_obj_name = name
        break

if target_obj_name:
    print(f"Found target object: {target_obj_name}")
    target_pose = np.copy(objs[target_obj_name][1])
    
    # 1. Move above the object
    above_pose = np.copy(target_pose)
    above_pose[2] += 0.15 # 15cm above
    
    # Orientation facing down
    rot = np.dot(as_rotation_matrix(quaternion(0, 0, 1, 0)),
                 np.array([[np.cos(np.pi / 2), -np.sin(np.pi / 2), 0],
                           [np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
                           [0, 0, 1]]))
    quat_wxyz = from_rotation_matrix(rot)
    grasping_quaternion = np.array([quat_wxyz.x, quat_wxyz.y, quat_wxyz.z, quat_wxyz.w])
    
    above_pose[3:] = grasping_quaternion
    print("Moving above object...")
    path = grasp_controller.get_path(above_pose, set_orientation=True)
    obs, _, _ = grasp_controller.execute_path(path, open_gripper=True)
    
    # 2. Move down to grasp
    grasp_pose = np.copy(target_pose)
    grasp_pose[2] += 0.02 # Slightly above the center of the object
    grasp_pose[3:] = grasping_quaternion
    print("Moving down to grasp...")
    path = grasp_controller.get_path(grasp_pose, set_orientation=True)
    obs, _, _ = grasp_controller.execute_path(path, open_gripper=True)
    
    # Save wrist image
    wrist_rgb = np.array(obs.wrist_rgb * 255, dtype='uint8')
    wrist_img = cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite('wrist_grasping.png', wrist_img)
    print("Saved wrist_grasping.png")
    
    # 3. Close gripper
    print("Closing gripper...")
    grasped = grasp_controller.grasp()
    print(f"Grasp status: {grasped}")
    
    # 4. Move up
    print("Moving up...")
    path = grasp_controller.get_path(above_pose, set_orientation=True)
    obs, _, _ = grasp_controller.execute_path(path, open_gripper=False)
    
    # 5. Move above red container
    print("Moving to red container...")
    drop_pose = np.copy(objs['waypoint3'][1])
    drop_pose[2] += 0.15
    drop_pose[3:] = grasping_quaternion
    path = grasp_controller.get_path(drop_pose, set_orientation=True)
    obs, _, _ = grasp_controller.execute_path(path, open_gripper=False)
    
    # 6. Release
    print("Releasing object...")
    grasp_controller.release()
    print("Grasp demo completed successfully!")

time.sleep(2)
grasp_controller.env.shutdown()
os._exit(0)
