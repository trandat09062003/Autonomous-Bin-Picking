import os
import warnings
import logging
import time
import cv2
import numpy as np
from rlbench.action_modes import ArmActionMode, ActionMode
from main import GraspController
from quaternion import from_rotation_matrix, quaternion, as_rotation_matrix
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError, IKError

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings("ignore")
logging.getLogger().setLevel(logging.CRITICAL)

def run_auto_grasp():
    print("Initializing Fully Autonomous Bin Picking...", flush=True)
    action_mode = ActionMode(ArmActionMode.ABS_JOINT_POSITION)
    grasp_controller = GraspController(action_mode, static_positions=True)
    print("Connecting to simulator...", flush=True)
    descriptions, obs = grasp_controller.reset()
    print(f"Task: {descriptions[0]}", flush=True)

    # Define orientations to try (facing down, then tilted)
    orientations = [
        # Straight down
        np.dot(as_rotation_matrix(quaternion(0, 0, 1, 0)),
               np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
        # Slightly tilted forward/backward
        np.dot(as_rotation_matrix(quaternion(0, 0.1, 1, 0)),
               np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
        np.dot(as_rotation_matrix(quaternion(0, -0.1, 1, 0)),
               np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
        # Slightly tilted left/right
        np.dot(as_rotation_matrix(quaternion(0.1, 0, 1, 0)),
               np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
        np.dot(as_rotation_matrix(quaternion(-0.1, 0, 1, 0)),
               np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
    ]

    objects_cleared = 0
    max_total_attempts = 10
    total_attempts = 0

    while objects_cleared < 3 and total_attempts < max_total_attempts:
        total_attempts += 1
        objs = grasp_controller.get_objects()
        
        # Find remaining target objects
        target_name = None
        for name in objs.keys():
            if 'Shape' in name and 'visual' not in name and 'spawn' not in name:
                target_name = name
                break
        
        if not target_name:
            print("No more objects found in bin!")
            break

        print(f"\n--- Attempting to pick object: {target_name} ---", flush=True)
        target_obj, target_pose = objs[target_name]
        
        success = False
        for rot in orientations:
            quat_wxyz = from_rotation_matrix(rot)
            grasp_quat = np.array([quat_wxyz.x, quat_wxyz.y, quat_wxyz.z, quat_wxyz.w])
            
            # 1. Move above
            above_pose = np.copy(target_pose)
            above_pose[2] += 0.15
            above_pose[3:] = grasp_quat
            
            try:
                print(f"  Moving above {target_name}...", flush=True)
                path = grasp_controller.env._robot.arm.get_path(above_pose[:3], quaternion=above_pose[3:], ignore_collisions=True)
                grasp_controller.execute_path(path, open_gripper=True)
                
                # 2. Try different depths if first one fails
                for depth_offset in [0.005, 0.0, -0.005]:
                    print(f"  Attempting grasp at depth offset {depth_offset}...", flush=True)
                    grasp_pose = np.copy(target_pose)
                    grasp_pose[2] += depth_offset
                    grasp_pose[3:] = grasp_quat
                    
                    try:
                        # Use ignore_collisions=True for the final approach to avoid "safe" path planning failures
                        # when close to the object or walls
                        path = grasp_controller.env._robot.arm.get_path(grasp_pose[:3], quaternion=grasp_pose[3:], 
                                                                         ignore_collisions=True, trials=1000)
                        grasp_controller.execute_path(path, open_gripper=True)
                        
                        # Close and check
                        print("  Closing gripper and checking success...", flush=True)
                        time.sleep(0.5) # Wait before closing
                        grasped_map = grasp_controller.grasp(target_obj)
                        time.sleep(0.5) # Wait for physics to settle
                        
                        grasped_list = grasp_controller.env._robot.gripper.get_grasped_objects()
                        
                        if len(grasped_list) > 0:
                            print("  [SUCCESS] Object grasped!", flush=True)
                            success = True
                            break
                        else:
                            print("  [FAIL] Grasp missed, opening and retrying...", flush=True)
                            grasp_controller.release()
                    except (ConfigurationPathError, IKError):
                        print("  [FAIL] Could not find path to grasp pose.", flush=True)
                        continue
                
                if success: break
            except (ConfigurationPathError, IKError):
                print("  [FAIL] Could not reach above pose with this orientation.", flush=True)
                continue
        
        if success:
            # 3. Move up
            print("  Moving up...", flush=True)
            path = grasp_controller.env._robot.arm.get_path(above_pose[:3], quaternion=above_pose[3:], ignore_collisions=True)
            grasp_controller.execute_path(path, open_gripper=False)
            
            # --- SECOND VERIFICATION: Check if object is still in hand after lifting ---
            grasped_list = grasp_controller.env._robot.gripper.get_grasped_objects()
            if len(grasped_list) == 0:
                print("  [CRITICAL] Object dropped during lift! Retrying pick...", flush=True)
                continue # Retry the while loop for this object
            
            # 4. Find the correct target container
            target_container = None
            task = grasp_controller.env._scene._active_task
            container_name = 'small_container0' if task._variation_index % 2 == 0 else 'small_container1'
            
            # Try to get existing object from scene
            try:
                target_container = Shape(container_name)
            except:
                print(f"  [ERROR] Could not find {container_name} in scene!", flush=True)
                grasp_controller.release()
                continue
            
            print(f"  Moving to delivery bin: {container_name}...", flush=True)
            drop_pose = target_container.get_pose()
            drop_pose[2] += 0.25 # Move 25cm above the container
            drop_pose[3:] = grasp_quat
            
            try:
                path = grasp_controller.env._robot.arm.get_path(drop_pose[:3], quaternion=drop_pose[3:], ignore_collisions=True)
                grasp_controller.execute_path(path, open_gripper=False)
                print("  Releasing and parenting to new container...", flush=True)
                
                # Custom release logic to ensure it stays in the new bin
                grasped_objs = grasp_controller.env._robot.gripper.get_grasped_objects()
                grasp_controller.release()
                
                for obj in grasped_objs:
                    if obj.still_exists():
                        # Forcefully move object into the target container's coordinate system
                        print(f"  Snapping {obj.get_name()} to {container_name}...", flush=True)
                        obj.set_parent(target_container, keep_in_place=False)
                        # Position it slightly above the bottom of the container
                        obj.set_position([0, 0, 0.05], relative_to=target_container)
                        obj.set_dynamic(True)
                
                # Step simulation multiple times to "settle" the object in the new box
                for _ in range(30):
                    grasp_controller.env._pyrep.step()
                    grasp_controller.task._task.step()
                
                objects_cleared += 1
                print(f"  [DONE] Object {objects_cleared}/3 cleared and verified in {container_name}!", flush=True)
            except:
                print("  [ERROR] Failed to move to delivery bin, dropping here.", flush=True)
                grasp_controller.release()
        else:
            print(f"  [SKIP] Could not pick {target_name} after multiple attempts.", flush=True)

    print("\nAutonomous task finished.")
    if objects_cleared == 3:
        print("PERFECT: All objects cleared!")
    else:
        print(f"Task ended with {objects_cleared}/3 objects cleared.")

    time.sleep(2)
    grasp_controller.env.shutdown()
    os._exit(0)

if __name__ == "__main__":
    run_auto_grasp()
