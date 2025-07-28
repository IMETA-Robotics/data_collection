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