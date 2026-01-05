import argparse
import os
import h5py
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import time
from y1_msg.msg import ArmJointPositionControl

def load_hdf5(dataset_dir, dataset_name):
  """Load eposide data from hdf5 file"""
  dataset_path = os.path.join(dataset_dir, dataset_name + ".hdf5")
  if not os.path.isfile(dataset_path):
    raise FileNotFoundError(f"Dataset file {dataset_path} not found.")
  print(f"Loading data from: {dataset_path}")

  with h5py.File(dataset_path, 'r') as root:
    # observation
    joint_position = root['/observation/state'][()]
    
    image_dict = dict()
    images_grp = root['/observation/images/']
    for cam_name in images_grp.keys():
        img_bytes_seq = images_grp[cam_name][()]
        # img_bytes_seq 可能是一个包含多帧字节的 NumPy 数组
        frames = []
        for frame_bytes in img_bytes_seq:
            buf = np.frombuffer(frame_bytes, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError(f"Failed to decode frame from camera '{cam_name}'.")
            frames.append(img)
        image_dict[cam_name] = frames
    
    # action
    action = root['/action'][()]
    
  return joint_position, image_dict, action
  
def main(args):
  print(f"args: {args}")
  rclpy.init()
  node = Node('replay_data')  # 直接创建节点实例

  dataset_dir = os.path.abspath(args['dataset_dir'])
  episode_idx = args['episode_idx']
  dataset_name = f"episode_{episode_idx}"
  
  # load data from hdf5 file
  joint_position, image_dict, action = load_hdf5(dataset_dir, dataset_name)
  
  left_arm_control = node.create_publisher(ArmJointPositionControl, '/master_arm_left/joint_states', 1)
  right_arm_control = node.create_publisher(ArmJointPositionControl, '/master_arm_right/joint_states', 1)

  left_arm_msg = ArmJointPositionControl()
  right_arm_msg = ArmJointPositionControl()
  action = np.array(action)

  # go to initial position
  init_position = action[0].tolist()
  print(f"Initial position: {init_position}")
  input("Press key [enter] to control robot to init pose!")
  if args['single_arm']:
    # single arm, default right arm
    right_arm_msg.header.stamp = node.get_clock().now().to_msg()
    right_arm_msg.joint_position = init_position[0:6]
    right_arm_msg.joint_velocity = 3
    right_arm_msg.gripper_stroke = init_position[6]
    right_arm_msg.gripper_velocity = 3
    right_arm_control.publish(right_arm_msg)
    print("single_arm")

  else:
    # dual arm
    left_arm_msg.header.stamp = node.get_clock().now().to_msg()
    left_arm_msg.joint_position = init_position[0:6]
    left_arm_msg.joint_velocity = 3
    left_arm_msg.gripper_stroke = init_position[6]
    left_arm_msg.gripper_velocity = 3
    left_arm_control.publish(left_arm_msg)

    right_arm_msg.header.stamp = node.get_clock().now().to_msg()
    right_arm_msg.joint_position = init_position[7:13]
    right_arm_msg.joint_velocity = 3
    right_arm_msg.gripper_stroke = init_position[13]
    right_arm_msg.gripper_velocity = 3
    right_arm_control.publish(right_arm_msg)
    print("dual_arm")

  time.sleep(3)
  length = action.shape[0]
  print("data length: ", length)
  input("Press key [enter] to replay data!")
  idx = 1
  fps = args['fps']
  while rclpy.ok() and idx < length:
    loop_start_time = time.perf_counter()
    
    print(f"Playing frame : {idx}")
    current_action = action[idx].tolist()
    # play action
    if args['single_arm']:
      # single arm, default right arm
        right_arm_msg.header.stamp = node.get_clock().now().to_msg()
        right_arm_msg.joint_position = current_action[0:6]
        right_arm_msg.joint_velocity = 3
        right_arm_msg.gripper_stroke = current_action[6]
        right_arm_msg.gripper_velocity = 3
        right_arm_control.publish(right_arm_msg)
    else:
       # dual arm
        left_arm_msg.header.stamp = node.get_clock().now().to_msg()
        left_arm_msg.joint_position = current_action[0:6]
        left_arm_msg.joint_velocity = 3
        left_arm_msg.gripper_stroke = current_action[6]
        left_arm_msg.gripper_velocity = 3
        left_arm_control.publish(left_arm_msg)

        right_arm_msg.header.stamp = node.get_clock().now().to_msg()
        right_arm_msg.joint_position = current_action[7:13]
        right_arm_msg.joint_velocity = 3
        right_arm_msg.gripper_stroke = current_action[13]
        right_arm_msg.gripper_velocity = 3
        right_arm_control.publish(right_arm_msg)
    idx += 1
    # 30 hz
    time.sleep(max(0, (1.0 / fps) - (time.perf_counter() - loop_start_time)))

  print("Finished playing episode.")
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  # parse cli args
  parser = argparse.ArgumentParser(description='Visualize episodes from a given dataset directory.')
  parser.add_argument('--dataset_dir', type=str, help='Path to the dataset directory.', required=True)
  parser.add_argument("--episode_idx", type=int, help="Episode index to visualize.", required=True)
  # deault dual arm, if you want to use single arm, set --single_arm True
  parser.add_argument("--single_arm", type=bool, help="single arm or dual arm", required=False, default=False)
  parser.add_argument("--fps", type=int, help="Episode index to visualize.", required=False, default=30)

  main(vars(parser.parse_args()))