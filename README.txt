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


###h5dump 用法:

仅显示文件结构（不显示数据）：
h5dump -H example.h5

列出文件中的所有对象：
h5dump -n example.h5

列出对象及其属性：
h5dump -n 1 example.h5