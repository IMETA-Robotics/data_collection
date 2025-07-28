"""
Script to convert h5 files to lerobot format.

example:
python -m scripts.convert_h5_to_lerobot \
  --config.raw-dir data/piper_place_and_place_0722/ \
  --config.repo-id piper/piper_place_and_place_0722 \
  --config.robot-type piper
"""

import tyro
import shutil
import h5py
import tqdm
from dataclasses import dataclass
from pathlib import Path
from lerobot.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset

@dataclass
class CovertConfig:
  raw_dir: Path
  repo_id: str
  robot_type: str = "IMETA_Y1"
  push_to_hub: bool = False
  
  use_video: bool = True
  tolerance_s: float = 0.0001
  image_writer_processes: int = 4
  image_writer_threads: int = 5
  
class LoadH5Dataset:
  def __init__(self, data_dir: Path) -> None:
    """
    Initialize the dataset for loading and processing HDF5 files contaning robot manipulation data.
    
    Arguments:
      data_dir (Path): The directory containing the HDF5 files.
    """
    self.data_dir  = data_dir.resolve()
    if not self.data_dir.exists():
      raise ValueError(f"Data directory {self.data_dir} does not exist.")
    
    hdf5_files = list(self.data_dir.glob("*.hdf5"))
    if len(hdf5_files) == 0:
      raise ValueError(f"Data directory {self.data_dir} is empty.")
    
    # sort
    self.datasets = sorted(
            hdf5_files,
            key=lambda p: int(p.stem.rsplit("_", 1)[1])
        )
    # print("hdf5_files: ", self.datasets)
    
  def __len__(self) -> int:
    """"Return the number of collected episodes."""
    return len(self.datasets)
  
  def get_item(self, index: int) -> dict:
    """Return the dataset at the given index."""
    episode_data = {}
    
    # load hdf5 file
    data_path = self.datasets[index]
    with h5py.File(data_path, 'r') as root:
      """observation"""
      # state(joint position)
      episode_data["state"] = root['/observation/state'][()]
      # joint velocity
      # episode_data["velocity"] = root['/observation/velocity'][()]
      # joint effort
      # episode_data["effort"] = root['/observation/effort'][()]
      # cameras
      cameras = {}
      for cam_name in root[f'/observation/images/'].keys():
        cameras[cam_name] = root[f'/observation/images/{cam_name}'][()]
        
      episode_data["cameras"] = cameras
      
      """action"""
      episode_data["action"] = root['/action'][()]
      
      # episode length
      episode_data["episode_length"] = len(root['/observation/state'][()])
    
    return episode_data
  
def create_empty_dataset(config: CovertConfig) -> LeRobotDataset:
  # features
  features = {}
  names = ["J1", "J2", "J3", "J4", "J5", "J6", "Gripper"]
  cameras = ["cam_right_wrist", "cam_front"]
  
  # state
  features["observation.state"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  # features["observation.effort"] = {
  #   "dtype": "float32",
  #   "shape": (7,),
  #   "names": names
  # }
  
  # features["observation.velocity"] = {
  #   "dtype": "float32",
  #   "shape": (7,),
  #   "names": names
  # }
  
  # images
  for cam in cameras:
    features[f"observation.images.{cam}"] = {
      "dtype": "video" if config.use_video else "image",
      "shape": (3, 480, 640),
      "names": ["channels", "height", "width"]
    }
  
  # action
  features["action"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  # 创建一个空的LeRobotDataset对象
  return LeRobotDataset.create(
      repo_id=config.repo_id,
      fps=50,
      robot_type=config.robot_type,
      features=features,
      use_videos=config.use_video,
      tolerance_s=config.tolerance_s,
      image_writer_processes=config.image_writer_processes,
      image_writer_threads=config.image_writer_threads * len(cameras),
  )
  
def h5_to_lerobot(config: CovertConfig, dataset: LeRobotDataset) -> LeRobotDataset:
  h5_dataset = LoadH5Dataset(config.raw_dir)
  for i in tqdm.tqdm(range(len(h5_dataset))):
    episode = h5_dataset.get_item(i)
    
    state = episode["state"]
    # velocity = episode["velocity"]
    # effort = episode["effort"]
    cameras = episode["cameras"]
    action =episode["action"]
    episode_length = episode["episode_length"]
    task = "piper_pick_and_place"
    
    for j in range(episode_length):
      frame = {
        "observation.state": state[j],
        # "observation.velocity": velocity[j],
        # "observation.effort": effort[j],
        "action": action[j],
      }
      
      for camera, image_arr in cameras.items():
        frame[f"observation.images.{camera}"] = image_arr[j]
        
      dataset.add_frame(frame, task)
      
    dataset.save_episode()
    
  return dataset
  
def convert_h5_to_lerobot(config: CovertConfig):
  # 如果目录存在，删除它
  if (HF_LEROBOT_HOME/config.repo_id).exists():
    print(f"{config.repo_id} already exists, removing...")
    shutil.rmtree(HF_LEROBOT_HOME/config.repo_id)
    
  # 创建LeRobotDataset
  dataset: LeRobotDataset = create_empty_dataset(config)
  
  # 开始转换
  dataset: LeRobotDataset = h5_to_lerobot(config, dataset)

  # 是否上传到HuggingFace
  if config.push_to_hub:
    dataset.push_to_hub(config.repo_id)

if __name__ == "__main__":
  tyro.cli(convert_h5_to_lerobot)
  