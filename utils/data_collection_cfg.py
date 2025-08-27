from dataclasses import dataclass, field, MISSING
from omegaconf import OmegaConf
from typing import Optional, List
from pathlib import Path

@dataclass
class Hdf5Cfg:
  save_as_h5: bool  = False
  """The data saved in hdf5 format."""
  
  dataset_dir: Optional[str] = None
  """Directory to save the dataset."""
  
  task_name: Optional[str] = None
  """Task name."""
  
  start_episode: Optional[int] = None
  
@dataclass
class LeRobotDatasetCfg:
  save_as_lerobot: bool = True
  """The data is saved in LeRobotDataset format."""
  
  repo_id: Optional[str] = None
  robot_type: str = "IMETA_Y1"
  push_to_hub: bool = False
  resume: bool = False
  root: Optional[Path] = None
  """Root directory where the dataset will be stored (e.g. 'dataset/path')."""
  
  use_video: bool = True
  tolerance_s: float = 0.0001
  num_image_writer_processes: int = 2
  """ Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses."""
  num_image_writer_threads_per_camera: int = 5
  """ Number of threads writing the frames as png images on disk, per camera.
    Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    Not enough threads might cause low camera fps."""

@dataclass
class RoboticArmCfg:
  """Configuration for robotic arm."""
  collection_type: Optional[str] = None
  """data collection type, can be 'one_master' or 'two_master' 
      or 'one_master_slave' or 'two_master_slave' """
  
  master_arm_right_topic: Optional[str] = None
  """Right master arm joint states topic."""
  
  puppet_arm_right_topic: Optional[str] = None
  """Right puppet arm joint states topic."""
  
  enable_left_arm: bool = False
  """Whether to enable left arm."""
  
  master_arm_left_topic: Optional[str] = None
  """Left master arm joint states topic."""
  
  puppet_arm_left_topic: Optional[str] = None
  """Left puppet arm joint states topic."""
  
  use_joint_velocity: bool = False
  """Whether to save joint velocity."""
  
  use_joint_effort: bool = False
  """Whether to save joint effort."""

@dataclass
class CameraCfg:
  """Configuration for camera."""
  
  camera_names: List[str] = field(default_factory=list)
  """The camera name in saved data"""
  
  img_right_topic: Optional[str] = None
  """right camera rgb image topic"""
  
  img_left_topic: Optional[str] = None
  """left camera rgb image topic"""
  
  img_front_topic: Optional[str] = None
  """front camera rgb image topic"""
  
  img_top_topic: Optional[str] = None
  """top camera rgb image topic"""
  
  depth_right_topic: Optional[str] = None
  """right camera depth image topic"""
  
  depth_left_topic: Optional[str] = None
  """left camera depth image topic"""
  
  depth_front_topic: Optional[str] = None
  """front camera depth image topic"""
  
  depth_top_topic: Optional[str] = None
  """top camera depth image topic"""

@dataclass
class DataCollectionCfg:
  """Configuration for robotic arm data collection."""

  task_description: str = MISSING
  
  num_episodes: int = 1
  """The number of collect expert demonstrations.default is 1."""
  
  save_rate: int = 30
  """The frequency of save data. default is 30."""
  
  hdf5_cfg: Hdf5Cfg = Hdf5Cfg()
  """Configuration for save data as hdf5."""
  
  lerobot_dataset_cfg: LeRobotDatasetCfg = LeRobotDatasetCfg()
  """Configuration for save data as lerobot dataset."""
  
  robotic_arm_cfg: RoboticArmCfg = RoboticArmCfg()
  """Configuration for robotic arm."""
  
  camera_cfg: CameraCfg = CameraCfg()
  """Configuration for camera."""

def build_config():
  """Parse configuration from command line arguments and YAML file."""
  # Step 1: Load base config from dataclass
  base_cfg = OmegaConf.structured(DataCollectionCfg)
  
  # Step 2: Load config from YAML file
  cli_cfg = OmegaConf.from_cli()
  if "config_file" in cli_cfg:
    yaml_cfg = OmegaConf.load(cli_cfg["config_file"])
    # print("yaml_cfg: ", yaml_cfg)
    base_cfg = OmegaConf.merge(base_cfg, yaml_cfg)
  else:
    raise ValueError("arg config_file=your_path must provided.")
  
  # Step 3: Merge CLI args
  if "config_file" in cli_cfg:
    del cli_cfg["config_file"]

  # print("cli_cfg: ", cli_cfg)
  cfg = OmegaConf.merge(base_cfg, cli_cfg)
  
  return cfg