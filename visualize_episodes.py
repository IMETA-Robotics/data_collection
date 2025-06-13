import argparse
import os
import h5py
import numpy as np
import cv2
import matplotlib.pyplot as plt

def load_hdf5(dataset_dir, dataset_name):
  """Load eposide data from hdf5 file"""
  dataset_path = os.path.join(dataset_dir, dataset_name + ".hdf5")
  if not os.path.isfile(dataset_path):
    raise FileNotFoundError(f"Dataset file {dataset_path} not found.")

  with h5py.File(dataset_path, 'r') as root:
    # obervation
    joint_position = root['/observation/qpos'][()]
    joint_velocity = root['/observation/qvel'][()]
    joint_effort = root['/observation/effort'][()]
    
    image_dict = dict()
    for cam_name in root[f'/observation/images/'].keys():
        image_dict[cam_name] = root[f'/observation/images/{cam_name}'][()]
    
    # action
    action = root['/action'][()]
    
  return joint_position, joint_velocity, joint_effort, image_dict, action

def visualize_joints(obs_list, command_list, plot_path):
  """visualize joint position or velocity or effort"""
  obs = np.array(obs_list)
  comannd = np.array(command_list)
  _, len_dim = obs.shape
  
  _, axs = plt.subplots(len_dim, 1, figsize=(len_dim, 2 * len_dim))
  
  # plot state
  for idx in range(len_dim):
    ax = axs[idx]
    ax.plot(obs[:, idx], label='observation')
    ax.set_title(f'Joint {idx+1}')
    ax.legend()
  
  # plot commnand
  for idx in range(len_dim):
    ax = axs[idx]
    ax.plot(comannd[:, idx], label='command')
    ax.legend()
    
  plt.tight_layout()
  plt.savefig(plot_path)
  print(f"Saved joints plot to: {plot_path}")
  plt.close()
  
def visualize_single_data(joint_list, label, plot_path):
  """visualize single data"""
  joint_data = np.array(joint_list)
  _, len_dim = joint_data.shape
  _, axs = plt.subplots(len_dim, 1, figsize=(len_dim, 2 * len_dim))
  
  # plot data 
  for idx in range(len_dim):
    ax = axs[idx]
    ax.plot(joint_data[:, idx], label=label)
    ax.set_title(f'Joint {idx+1}')
    ax.legend()
    
  plt.tight_layout()
  plt.savefig(plot_path)
  print(f"Saved {label} data plot to: {plot_path}")
  plt.close()

def save_video(image_dict: dict, video_fps: int, video_path: str):
  """visualize images and save to video"""
  cam_names = list(image_dict.keys())
  all_cam_images = []
  for cam_name in cam_names:
    all_cam_images.append(image_dict[cam_name])
  all_cam_images = np.concatenate(all_cam_images, axis=2)
  
  num_image, h , w, _ = all_cam_images.shape
  out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), video_fps, (w, h))
  if not out.isOpened():
    raise FileNotFoundError("cv2.VideoWriter is not opened!")
  
  print(f"Start to save video, fps:{video_fps}, frames:{num_image}")
  for t in range(num_image):
    image = all_cam_images[t]
    # opencv default is brg, change brg to rgb
    image = image[:, :, [2, 1, 0]]
    out.write(image)
    
  out.release()
  print(f"Success save video to: {video_path}.")
  
def main(args):
  dataset_dir = os.path.abspath(args['dataset_dir'])
  episode_idx = args['episode_idx']
  dataset_name = f"episode_{episode_idx}"
  
  # load data from hdf5 file
  joint_position, joint_velocity, joint_effort, image_dict, action = load_hdf5(dataset_dir, dataset_name)
  
  # save video
  save_video(image_dict, 50, video_path=os.path.join(dataset_dir, dataset_name + "_video.mp4"))
  # visual joint position and action
  visualize_joints(joint_position, action, 
                   plot_path=os.path.join(dataset_dir, dataset_name + "_joint_postion_and_action.png"))
  # visual single joint velocity
  visualize_single_data(joint_velocity, 'joint_velocity', 
                        plot_path=os.path.join(dataset_dir, dataset_name + "_joint_velocity.png"))
  # visual single joint effort
  visualize_single_data(joint_effort, 'joint_effort', 
                        plot_path=os.path.join(dataset_dir, dataset_name + "_joint_effort.png"))
  # visual error of joint position and action
  visualize_single_data(action - joint_position, 'joint_position_and_action_error', 
                        plot_path=os.path.join(dataset_dir, dataset_name + '_joint_position_and_action_error.png'))

if __name__ == "__main__":
  # parse cli args
  parser = argparse.ArgumentParser(description='Visualize episodes from a given dataset directory.')
  parser.add_argument('--dataset_dir', type=str, help='Path to the dataset directory.', required=True)
  parser.add_argument("--episode_idx", type=int, help="Episode index to visualize.", required=True)
  main(vars(parser.parse_args()))