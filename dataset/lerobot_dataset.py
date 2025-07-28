""""
  Save datsets to lerobot datasets format.
"""

import shutil
import time
import numpy as np
import os
from utils.data_collection_cfg import DataCollectionCfg
from lerobot.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class LRDataset:
  def __init__(self, cfg: DataCollectionCfg):
    self.cfg = cfg
    lr_cfg = cfg.lerobot_dataset_cfg
    if lr_cfg.resume:
      # 在原有的repo_id数据集中添加新数据
      self.lr_dataset = LeRobotDataset(lr_cfg.repo_id, root=lr_cfg.root)

      self.lr_dataset.start_image_writer(
          num_processes=lr_cfg.num_image_writer_processes,
          num_threads=lr_cfg.num_image_writer_threads_per_camera * len(cfg.camera_cfg.camera_names),
      )
    else:
      # Create empty dataset
      self.lr_dataset = self.create_empty_dataset(self.cfg)
    
  def __del__(self):
    self.lr_dataset.stop_image_writer()
    if self.cfg.lerobot_dataset_cfg.push_to_hub:
      self.lr_dataset.push_to_hub(self.cfg.lerobot_dataset_cfg.repo_id)
    
  def create_empty_dataset(self, cfg: DataCollectionCfg):
    """init lerobot dataset"""
    
    lerobot_cfg = cfg.lerobot_dataset_cfg
    # 如果目录存在，删除它
    if lerobot_cfg.root is None:
      if (HF_LEROBOT_HOME/lerobot_cfg.repo_id).exists():
        print(f"{lerobot_cfg.repo_id} already exists, removing...")
        shutil.rmtree(HF_LEROBOT_HOME/lerobot_cfg.repo_id)
    else:
      abs_root = lerobot_cfg.root.resolve()
      if abs_root.exists():
        print(f"{abs_root} already exists, removing...")
        shutil.rmtree(abs_root)
      
    # features
    features = {}
    names = ["J1", "J2", "J3", "J4", "J5", "J6", "Gripper"]
    cameras = cfg.camera_cfg.camera_names
    
    if (cfg.robotif_arm_cfg.enable_left_arm):
      shape = (14,)
    else:
      shape = (7,)
      
    # state    
    features["observation.state"] = {
      "dtype": "float32",
      "shape": shape,
      "names": names
    }
    
    if self.cfg.robotif_arm_cfg.use_joint_effort:
      features["observation.effort"] = {
        "dtype": "float32",
        "shape": shape,
        "names": names
      }
    
    if self.cfg.robotif_arm_cfg.use_joint_velocity:
      features["observation.velocity"] = {
        "dtype": "float32",
        "shape": shape,
        "names": names
      }
    
    # images
    for cam in cameras:
      features[f"observation.images.{cam}"] = {
        "dtype": "video" if lerobot_cfg.use_video else "image",
        "shape": (3, 480, 640),
        "names": ["channels", "height", "width"]
      }
    
    # action
    features["action"] = {
      "dtype": "float32",
      "shape": shape,
      "names": names
    }
    
    # 创建一个空的LeRobotDataset对象
    return LeRobotDataset.create(
        repo_id=lerobot_cfg.repo_id,
        fps=cfg.save_rate,
        robot_type=lerobot_cfg.robot_type,
        features=features,
        use_videos=lerobot_cfg.use_video,
        tolerance_s=lerobot_cfg.tolerance_s,
        image_writer_processes=lerobot_cfg.num_image_writer_processes,
        image_writer_threads=lerobot_cfg.num_image_writer_threads_per_camera * len(cameras),
    )
    
  def save_to_lerobot(self, timesteps, actions):
    """save one episode"""
    len_episode = len(actions)
    if (len_episode == 0): 
      print("LerobotDataset not save, check this episode data is empty!")
      return
    
    print(f"episode length: {len_episode}. Start save to lerobot dataset format file!")
    
    start_time = time.time()
    for j in range(len_episode):
      ts = timesteps[j]
      action = np.array(actions[j], dtype=np.float32)
      state = np.array(ts.observation["state"], dtype=np.float32)
      
      frame = {
        "observation.state": state,
        "action": action,
      }
      
      if self.cfg.robotif_arm_cfg.use_joint_velocity:
        velocity = np.array(ts.observation["velocity"], dtype=np.float32)
        frame["observation.velocity"] = velocity
      if self.cfg.robotif_arm_cfg.use_joint_effort:
        effort = np.array(ts.observation["effort"], dtype=np.float32)
        frame["observation.effort"] = effort
      
      for cam_name in self.cfg.camera_cfg.camera_names:
        frame[f"observation.images.{cam_name}"] = ts.observation["cameras"][cam_name]
        
      self.lr_dataset.add_frame(frame, self.cfg.task_description)
      
    self.lr_dataset.save_episode()
    print(f"Save episode to {HF_LEROBOT_HOME/self.cfg.lerobot_dataset_cfg.repo_id}.")
    print(f"Save time: {time.time() - start_time:.1f} secs.")