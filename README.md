# 1. Software installation
Choose one of the following two methods
## 1.1 Conda:
    conda create -n imeta_data_collection python=3.8.10
    conda activate imeta_data_collection
    
    pip install numpy
    pip install rospkg
    pip install opencv-python
    pip install omegaconf
    pip install pynput
    pip install dm_env
    pip install h5py
    pip install matplotlib

## 1.2 or create conda environment:
conda env create -f conda_environment.yml

# 2. Data collection:
## 2.1 collecte data with hdf5 format

source the ros env first, because we use rosmsg in data collection:
`cd y1_ros/`
`source devel/setup.bash`

if you have only one master arm:
`python -m scripts.record_data config_file=cfg/one_master.yaml`

if you have two master arms:
`python -m scripts.record_data config_file=cfg/two_master.yaml`

if you have one master and one slave arm:
`python -m scripts.record_data config_file=cfg/one_master_slave.yaml`

if you have two master and two slave arm:
`python -m scripts.record_data config_file=cfg/two_master_slave.yaml`

## 参数配置
以two_master_slave.yaml配置为例解释:

| 参数 | 值 |
|---|---|
| **task_description** | "pick and place" |
| **num_episodes** | 100 |
| **save_rate** | 30 |

---

### hdf5_cfg

| 参数 | 值 |
|---|---|
| save_as_h5 | True |
| dataset_dir | "data/" |
| task_name | "two_arm_teleop" |

---

### lerobot_dataset_cfg

| 参数 | 值 |
|---|---|
| save_as_lerobot | True |
| repo_id | "y1/two_arm_teleop" |
| robot_type | "y1" |

---

### robotic_arm_cfg

| 参数 | 值 |
|---|---|
| collection_type | "two_master_slave" |
| master_arm_right_topic | "/master_arm_right/joint_states" |
| puppet_arm_right_topic | "/puppet_arm_right/joint_states" |
| master_arm_left_topic | "/master_arm_left/joint_states" |
| puppet_arm_left_topic | "/puppet_arm_left/joint_states" |

---

### camera_cfg

| 参数 | 值 |
|---|---|
| camera_names | ["cam_right_wrist", "cam_front", "came_left_wrist"] |
| img_right_topic | "/camera_right/color/image_raw" |
| img_left_topic | "/camera_left/color/image_raw" |
| img_front_topic | "/camera_front/color/image_raw" |


## 2.2. collecte data with LeRobotDataset V2.1 format

### 2.2.1 install lerobot in current python env "imeta_data_collection"
  if you want use LeRobotDataset V2.1, install lerobot:
`git clone https://github.com/imeta-lab/imeta_lerobot.git`
`cd imeta_lerobot/lerobot`
`pip install -e .`

### 2.2.2 set save_as_lerobot to True in yaml file
`save_as_lerobot: True`

### 2.2.3 use the program in 2.1 to collect data
`python -m scripts.record_data config_file=cfg/your_example.yaml`

### 2.2.4 or convert hdf5 data to lerobot (Recommended)
`python -m scripts.convert_h5_to_lerobot --config.raw-dir=data/pick_and_place/ --config.repo-id=y1/pick_and_place`

# 3. Data visualization
## 3.1 visualize hdf5 data:
python -m scripts.visualize_h5_episode --dataset_dir data/pick_and_place/ --episode_idx 0

## 3.2 visualize lerobot data:
Refer to the script of lerobot: 
`lerobot.scripts.visualize_dataset.py`

# Data Collection KeyBoard
S Start collect current episode
E End and save current episode
R Restart collect cuurent episode
Q exit

# you can use h5dump to view hdf5 data: 
Show only the file structure (no data):
`h5dump -H example.h5`