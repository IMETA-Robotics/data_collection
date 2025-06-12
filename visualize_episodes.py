import argparse
import os
import h5py
import numpy as np
import cv2

def load_hdf5(dataset_dir, dataset_name):
  dataset_path = os.path.join(dataset_dir, dataset_name + ".hdf5")
  if not os.path.isfile(dataset_path):
    raise FileNotFoundError(f"Dataset file {dataset_path} not found.")

  with h5py.File(dataset_path, 'r') as root:
    # is_sim = root.attrs['sim']
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

def save_video(image_dict: dict, video_fps: int, video_path: str):
  cam_names = list(image_dict.keys())
  all_cam_images = []
  for cam_name in cam_names:
    all_cam_images.append(image_dict[cam_name])
  all_cam_images = np.concatenate(all_cam_images, axis=2)
  
  num_image, h , w, _ = all_cam_images.shape
  out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), video_fps, (w, h))
  print(video_path, "is opened? ", out.isOpened(), ",num frames:", num_image)
  for t in range(num_image):
    image = all_cam_images[t]
    image = image[:, :, [2, 1, 0]] # covert to RGB
    out.write(image)
    
  out.release()
  print(f"Saved video to: {video_path}")
  
def main(args):
  dataset_dir = os.path.abspath(args['dataset_dir'])
  episode_index = args['episode_index']
  dataset_name = f"episode_{episode_index}"
  
  # load data from hdf5 file
  joint_position, joint_velocity, joint_effort, image_dict, action = load_hdf5(dataset_dir, dataset_name)
  
  # save video
  save_video(image_dict, 50, video_path=os.path.join(dataset_dir, dataset_name + "_video.mp4"))
  


if __name__ == "__main__":
  # parse cli args
  parser = argparse.ArgumentParser(description='Visualize episodes from a given dataset directory.')
  parser.add_argument('--dataset_dir', type=str, help='Path to the dataset directory.', required=True)
  parser.add_argument("--episode_index", type=int, help="Episode index to visualize.", required=True)
  main(vars(parser.parse_args()))