from dataclasses import dataclass, field, MISSING
from omegaconf import OmegaConf
from typing import Optional, List

@dataclass
class RoboticArmCfg:
  """Configuration for robotic arm."""
  
  master_arm_right_topic: str = "/master_arm_right/joint_states"
  """Right master arm joint states topic."""
  
  puppet_arm_right_topic: str = "/puppet_arm_right/joint_states"
  """Right puppet arm joint states topic."""
  
  enable_left_arm: bool = False
  """Whether to enable left arm."""
  
  master_arm_left_topic: Optional[str] = None
  """Left master arm joint states topic."""
  
  puppet_arm_left_topic: Optional[str] = None
  """Left puppet arm joint states topic."""

@dataclass
class CameraCfg:
  """Configuration for camera."""
  
  camera_names: List[str] = field(default_factory=list)
  """The camera name in saved data"""
  
  img_right_topic: str = "/camera_right/color/image_raw"
  """right camera rgb image topic"""
  
  img_left_topic: Optional[str] = None
  """left camera rgb image topic"""
  
  img_front_topic: Optional[str] = None
  """front camera rgb image topic"""
  
  img_top_topic: Optional[str] = None
  """top camera rgb image topic"""
  
  enable_depth_image: bool = False
  """Whether to enable depth image"""
  
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
  
  dataset_dir: str = MISSING
  """Directory to save the dataset."""
  
  task_name: str = MISSING
  """Task name."""
  
  start_episode: Optional[int] = None
  
  num_episodes: int = 1
  """The number of collect expert demonstrations.default is 1."""
  
  save_rate: int = 200
  """The frequency of save data. default is 200."""
  
  robotif_arm_cfg: RoboticArmCfg = RoboticArmCfg()
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