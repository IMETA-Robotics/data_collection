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
conda env create -f conda_environment.yml

if you want use lerobot datasets, install lerobot:
git clone https://github.com/imeta-lab/imeta_lerobot.git
cd imeta_lerobot/lerobot
pip install -e .

### Usage:
collecte data(support hdf5 and lerobot format):
python -m scripts.record_data config_file=cfg/one_master_slave.yaml

convert hdf5 data to lerobot:
python -m scripts.convert_h5_to_lerobot --config.raw-dir=data/piper_place_and_place_0729/ --config.repo-id=piper/piper_place_and_place_0729

visualize hdf5 data:
python -m scripts.visualize_episodes --dataset_dir data/piper_pick_and_place/ --episode_idx 0

visualize lerobot data:


### TODO:
1. 一次多条数据采集的时候，可以取消中间某条数据，重新采集该数据

### h5dump 用法:

仅显示文件结构（不显示数据）：
h5dump -H example.h5

列出文件中的所有对象：
h5dump -n example.h5

列出对象及其属性：
h5dump -n 1 example.h5