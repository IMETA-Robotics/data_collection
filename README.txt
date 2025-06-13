### Software installation - Conda:

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

### Create conda environment:
(recommend):
mamba env create -f conda_environment.yml

but you can use conda as well:
conda env create -f conda_environment.yml

### Useage:
collecte data:
python record_data.py config_file=config/one_master_slave.yaml num_episodes=30

visualize data:
python visualize_episodes.py --dataset_dir data/piper_pick_and_place/ --episode_idx 0

### TODO:
1. 一次多条数据采集的时候，可以取消中间某条数据，重新采集该数据

### h5dump 用法:

仅显示文件结构（不显示数据）：
h5dump -H example.h5

列出文件中的所有对象：
h5dump -n example.h5

列出对象及其属性：
h5dump -n 1 example.h5