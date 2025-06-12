from pynput import keyboard
import os

# 全局状态变量
_e_pressed = False

def _on_e_press(key):
    global _e_pressed
    if hasattr(key, "char") and key.char.lower() == 'e':
      _e_pressed = True

def start_e_listener():
    """启动后台监听 E 键（非阻塞）"""
    listener = keyboard.Listener(on_press=_on_e_press)
    listener.daemon = True
    listener.start()
    return listener

def is_e_pressed():
    """查询是否按下 E 键，按下后自动重置状态"""
    global _e_pressed
    if _e_pressed:
        _e_pressed = False
        return True
    return False

def wait_for_s():
    """阻塞等待 S or s 键按下后继续"""
    def on_s_press(key):
        if hasattr(key, "char") and key.char.lower() == 's':
            print("Detected 'S', continue…")
            return False  # 停止监听，结束阻塞

    with keyboard.Listener(on_press=on_s_press) as listener:
        listener.join() 

def get_auto_index(dataset_dir, start_episode=0, dataset_name_prefix = '', data_suffix = 'hdf5'):
    """
      获取数据集目录中下一个可用的片段索引
      
      参数：
          dataset_dir: 数据集目录
          dataset_name_prefix: 数据集名称前缀
          data_suffix: 数据文件扩展名
          
      返回：
          int: 下一个可用的片段索引
    """
    # 设置最大索引值为1000，防止无限循环
    max_idx = 1000
    
    # 如果数据集目录不存在，创建该目录
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    
    # 从0到1000遍历可能的索引值
    for i in range(max_idx+1):
        # 构造可能的文件名，例如：'episode_0.hdf5', 'episode_1.hdf5' 等
        # 检查该文件是否已经存在
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            # 如果文件不存在，返回这个索引值
            return i
    
    # 如果遍历完1000个索引都没找到可用的，抛出异常
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")
