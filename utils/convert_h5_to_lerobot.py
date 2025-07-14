"""
Script to convert h5 files to lerobot format.


"""

import tyro
import shutil
from dataclasses import dataclass
from pathlib import Path
from lerobot.common.constants import HF_LEROBOT_HOME
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset


@dataclass
class CovertConfig:
  raw_dir: Path
  repo_id: str
  robot_type: str
  push_to_hub: bool = False
  
  use_video: bool = True
  tolerance_s: float = 0.0001
  image_writer_processes: int = 1
  image_writer_threads: int = 4
  
class H5Dataset:
  def __init__(self, convert_config: CovertConfig):
    
  
def create_empty_dataset(convert_config: CovertConfig) -> LeRobotDataset:
  # features
  features = {}
  names = ["J1", "J2", "J3", "J4", "J5", "J6", "Gripper"]
  cameras = ["cam_right_wrist"]
  
  # state
  features["observation.state"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  features["observation.effort"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  features["observation.velocity"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  # images
  for cam in cameras:
    features[f"observation.images.{cam}"] = {
      "dtype": "video" if convert_config.use_video else "image",
      "shape": (3, 480, 720),
      "names": ["channels", "height", "width"]
    }
  
  # action
  features["action"] = {
    "dtype": "float32",
    "shape": (7,),
    "names": names
  }
  
  # 创建一个空的LeRobotDataset对象
  return LeRobotDataset(
      repo_id=convert_config.repo_id,
      fps=50,
      robot_type=convert_config.robot_type,
      features=features,
      use_video=convert_config.use_video,
      tolerance_s=convert_config.tolerance_s,
      image_writer_processes=convert_config.image_writer_processes,
      image_writer_threads=convert_config.image_writer_threads * len(cameras),
  )
  
def h5_to_lerobot(convert_config: CovertConfig, dataset: LeRobotDataset) -> LeRobotDataset:
  
  

def convert_h5_to_lerobot(convert_config: CovertConfig):
  # 如果目录存在，删除它
  if (HF_LEROBOT_HOME/convert_config.repo_id).exists():
    print(f"{convert_config.repo_id} already exists, removing...")
    shutil.rmtree(HF_LEROBOT_HOME/convert_config.repo_id)
    
  # 创建LeRobotDataset
  create_empty_dataset(convert_config)
  
  # 开始转换
  h5_to_lerobot(convert_config)
  

if __name__ == "__main__":
  tyro.cli(convert_h5_to_lerobot)