from utils.data_collection_cfg import DataCollectionCfg
from dataset.hdf5_dataset import H5Dataset
from y1_msg.msg import ArmJointState
from sensor_msgs.msg import Image
import collections
import dm_env
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from utils.key_handle import init_keyboard_listener

import rclpy
from rclpy.node import Node

class DataCollection(Node):
  def __init__(self, cfg: DataCollectionCfg):
    # 确保rclpy已初始化
    if not rclpy.ok():
      rclpy.init()

    super().__init__('data_collection_node')
    self.cfg = cfg
    self.bridge = CvBridge()
    self.master_arm_right_state = None
    self.master_arm_left_state = None
    self.puppet_arm_right_state = None
    self.puppet_arm_left_state = None
    self.img_dict = {"cam_right_wrist": None,
                     "cam_left_wrist": None,
                     "cam_high": None,
                     "cam_low": None}
    self.depth_dict = {}

    # 初始化订阅
    self.init_subscriptions()
    time.sleep(1)

  def destroy(self):
    self.destroy_node()
    rclpy.shutdown()
    
  def init_subscriptions(self):
    # subscribe robotic arm data
    robotic_arm_cfg = self.cfg.robotic_arm_cfg
    if robotic_arm_cfg.master_arm_right_topic is not None:
        self.create_subscription(
            ArmJointState,
            robotic_arm_cfg.master_arm_right_topic,
            self.master_arm_right_callback,
            1)
    
    if robotic_arm_cfg.master_arm_left_topic is not None:
        self.create_subscription(
            ArmJointState,
            robotic_arm_cfg.master_arm_left_topic,
            self.master_arm_left_callback,
            1)
    
    if robotic_arm_cfg.puppet_arm_right_topic is not None:
        self.create_subscription(
            ArmJointState,
            robotic_arm_cfg.puppet_arm_right_topic,
            self.puppet_arm_right_callback,
            1)
        
    if robotic_arm_cfg.puppet_arm_left_topic is not None:
        self.create_subscription(
            ArmJointState,
            robotic_arm_cfg.puppet_arm_left_topic,
            self.puppet_arm_left_callback,
            1)
      
    # subscribe camera rgb data
    camera_cfg = self.cfg.camera_cfg
    if camera_cfg.img_right_topic:
        # right arm wrist camera rgb image
        self.create_subscription(
            Image, camera_cfg.img_right_topic, self.img_right_callback, 1)
    
    if camera_cfg.img_left_topic:
        # left arm wrist camera rgb image
        self.create_subscription(
            Image, camera_cfg.img_left_topic, self.img_left_callback, 1)
      
    if camera_cfg.img_high_topic:
        # high external camera rgb image
        self.create_subscription(
            Image, camera_cfg.img_high_topic, self.img_high_callback, 1)
      
    if camera_cfg.img_low_topic:
        # low external camera rgb image
        self.create_subscription(
            Image, camera_cfg.img_low_topic, self.img_low_callback, 1)
      
    # subscribe camera depth data
    if camera_cfg.depth_right_topic:
        # right arm wrist camera depth image
        self.create_subscription(
            Image, camera_cfg.depth_right_topic, self.depth_right_callback, 1)
    
    if camera_cfg.depth_left_topic:
        # left arm wrist camera depth image
        self.create_subscription(
            Image, camera_cfg.depth_left_topic, self.depth_left_callback, 1)
      
    if camera_cfg.depth_high_topic:
        # high external camera depth image
        self.create_subscription(
            Image, camera_cfg.depth_high_topic, self.depth_high_callback, 1)
      
    if camera_cfg.depth_low_topic:
        # low external camera depth image
        self.create_subscription(
            Image, camera_cfg.depth_low_topic, self.depth_low_callback, 1)

  def master_arm_right_callback(self, msg: ArmJointState):
    self.master_arm_right_state = msg 
    
  def puppet_arm_right_callback(self, msg: ArmJointState):
    self.puppet_arm_right_state = msg
    
  def master_arm_left_callback(self, msg: ArmJointState):
    self.master_arm_left_state = msg
    
  def puppet_arm_left_callback(self, msg: ArmJointState):
    self.puppet_arm_left_state = msg
    
  def img_right_callback(self, msg: Image):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    # 编码为 JPEG 格式的二进制数据
    ret, buf = cv2.imencode('.jpg', cv_image)
    if ret:
      self.img_dict["cam_right_wrist"] = buf.tobytes()
    
  def img_left_callback(self, msg: Image):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    # 编码为 JPEG 格式的二进制数据
    ret, buf = cv2.imencode('.jpg', cv_image)
    if ret:
      self.img_dict["cam_left_wrist"] = buf.tobytes()
    
  def img_high_callback(self, msg: Image):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    # 编码为 JPEG 格式的二进制数据
    ret, buf = cv2.imencode('.jpg', cv_image)
    if ret:
      self.img_dict["cam_high"] = buf.tobytes()
    
  def img_low_callback(self, msg: Image):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    # 编码为 JPEG 格式的二进制数据
    ret, buf = cv2.imencode('.jpg', cv_image)
    if ret:
      self.img_dict["cam_low"] = buf.tobytes()
    
  def depth_right_callback(self, msg: Image):
    self.depth_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_left_callback(self, msg: Image):
    self.depth_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_high_callback(self, msg: Image):
    self.depth_high = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_low_callback(self, msg: Image):
    self.depth_low = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def get_frame(self):
    """Get all the data of a frame"""
    frame = {}
    
    # TODO: 要对数据做时间同步吗？
    if self.cfg.robotic_arm_cfg.collection_type == "one_master":
      # 单臂末端2合1数采，默认为右臂
      if self.master_arm_right_state is None:
        print("Not receive master_arm_right_state data!")
        return False
      
      frame["state"] = np.array(self.master_arm_right_state.joint_position)
      frame["action"] = np.array(self.master_arm_right_state.joint_position)
      if self.cfg.robotic_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.array(self.master_arm_right_state.joint_velocity)
      if self.cfg.robotic_arm_cfg.use_joint_effort:
        frame["effort"] = np.array(self.master_arm_right_state.joint_effort)
        
    elif self.cfg.robotic_arm_cfg.collection_type == "two_master":
      # 双臂末端2合1数采
      if self.master_arm_right_state is None:
        print("Not receive master_arm_right_state data!")
        return False
      if self.master_arm_left_state is None:
        print("Not receive master_arm_left_state data!")
        return False
      
      frame["state"] = np.concatenate([self.master_arm_left_state.joint_position,
                                 self.master_arm_right_state.joint_position])
      frame["action"] = np.concatenate([self.master_arm_left_state.joint_position,
                                 self.master_arm_right_state.joint_position])
      if self.cfg.robotic_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.concatenate([self.master_arm_left_state.joint_velocity,
                                 self.master_arm_right_state.joint_velocity])
      if self.cfg.robotic_arm_cfg.use_joint_effort:
        frame["effort"] = np.concatenate([self.master_arm_left_state.joint_effort,
                                 self.master_arm_right_state.joint_effort])
    elif self.cfg.robotic_arm_cfg.collection_type == "one_master_slave":
      # 单臂主从数采, 默认为右臂
      if self.master_arm_right_state is None:
        print("Not receive master_arm_right_state data!")
        return False
      if self.puppet_arm_right_state is None:
        print("Not receive puppet_arm_right_state data!")
        return False
      
      frame["state"] = np.array(self.puppet_arm_right_state.joint_position)
      frame["action"] = self.master_arm_right_state.joint_position
      if self.cfg.robotic_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.array(self.puppet_arm_right_state.joint_velocity)
      if self.cfg.robotic_arm_cfg.use_joint_effort:
        frame["effort"] = np.array(self.puppet_arm_right_state.joint_effort)
    
    elif self.cfg.robotic_arm_cfg.collection_type == "two_master_slave":
      # 双臂主从数采
      if self.master_arm_right_state is None:
        print("Not receive master_arm_right_state data!")
        return False
      
      if self.master_arm_left_state is None:
        print("Not receive master_arm_left_state data!")
        return False
      
      if self.puppet_arm_right_state is None:
        print("Not receive puppet_arm_right_state data!")
        return False
      
      if self.puppet_arm_left_state is None:
        print("Not receive puppet_arm_left_state data!")
        return False
      
      frame["state"] = np.concatenate([self.puppet_arm_left_state.joint_position,
                                      self.puppet_arm_right_state.joint_position])
      frame["action"] = np.concatenate([self.master_arm_left_state.joint_position,
                                 self.master_arm_right_state.joint_position])
      if self.cfg.robotic_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.concatenate([self.puppet_arm_left_state.joint_velocity,
                                              self.puppet_arm_right_state.joint_velocity])
      if self.cfg.robotic_arm_cfg.use_joint_effort:
        frame["effort"] = np.concatenate([self.puppet_arm_left_state.joint_effort,
                                          self.puppet_arm_right_state.joint_effort])
      
    else:
      print(f"Unsupport collection type: {self.cfg.robotic_arm_cfg.collection_type}")
      exit(1)
    
    cameras = {}
    for cam_name in self.cfg.camera_cfg.camera_names:
      if cam_name == "cam_right_wrist" and self.img_dict["cam_right_wrist"] is None:
        print("Not receive right image data!")
        return False
      if cam_name == "cam_left_wrist" and self.img_dict["cam_left_wrist"] is None:
        print("Not receive left image data!")
        return False
      if cam_name == "cam_high" and self.img_dict["cam_high"] is None:
        print("Not receive high image data!")
        return False
      if cam_name == "cam_low" and self.img_dict["cam_low"] is None:
        print("Not receive low image data!")
        return False
      
      cameras[cam_name] = self.img_dict[cam_name]
      
    frame["cameras"] = cameras
     
    return frame
    
  def collect(self, events: dict):
    target_period = 1.0 / self.cfg.save_rate

    timesteps = []
    actions = []
    first_step = True
    get_frame = False
    
    while rclpy.ok():
      loop_start_time = time.perf_counter()
      rclpy.spin_once(self, timeout_sec=0.01)

      if events["finish_current_recording"]:
        events["finish_current_recording"] = False
        break
      
      if events["stop_recording"]:
        print("Exit program!")
        rclpy.shutdown()
        exit(1)
      
      frame = self.get_frame() 
      if not frame:
        print("get frame failed!")
        # Maintain frequency
        time.sleep(max(0, target_period - (time.perf_counter() - loop_start_time)))
        continue

      elif get_frame is False:
        print("get frame success!")
        get_frame = True
      
      # observation
      obs = collections.OrderedDict()
      
      # state
      obs["state"] = frame["state"]

      if self.cfg.robotic_arm_cfg.use_joint_velocity:
        obs["velocity"] = frame["velocity"]
      if self.cfg.robotic_arm_cfg.use_joint_effort:
        obs["effort"] = frame["effort"]
      
      # camera image data
      obs['cameras'] = frame["cameras"]
      
      if first_step:
        # not append action in first step
        first_step = False
        ts = dm_env.TimeStep(
          step_type=dm_env.StepType.FIRST,
          reward=None,
          discount=None,
          observation=obs
        )
        timesteps.append(ts)
      else:
        ts = dm_env.TimeStep(
          step_type=dm_env.StepType.MID,
          reward=None,
          discount=None,
          observation=obs
        )
        
        timesteps.append(ts)
        actions.append(frame["action"])
      
      # Maintain frequency
      time.sleep(max(0, target_period - (time.perf_counter() - loop_start_time)))
      
    # ctrl-c exit the program 
    if not rclpy.ok(): 
        print("Exit program!")
        exit(1)
      
    return timesteps, actions
    
  def run(self):
    if self.cfg.num_episodes < 1:
      raise ValueError("num_episodes must be greater than 0")
    
    h5_dataset = H5Dataset(self.cfg)
    
    # start_e_listener()
    listener, events = init_keyboard_listener()
    if listener is None:
      exit(1)
      
    # 一共可以采集num_episodes条数据
    count = 0
    while count < self.cfg.num_episodes:
    # for i in range(self.cfg.num_episodes):
      # TODO: 每次采集前让机械臂回初始零位？
      print(f"Press key [S] to start record {count + 1}th episode!")
      while events["start_recording"] is False:
        if events["stop_recording"]:
          print("Exit program!")
          exit(1)
          
      events["start_recording"] = False
      print("Start record data! Press [E] to finish record current episode, or [R] to rerecord!")
      # collecte one episode
      timesteps, actions = self.collect(events)
      
      if len(actions) == 0:
        print(f"Data is empty, rerecord {count + 1}th episode!")
        continue
      
      if events["rerecord_episode"]:
        print(f"Rerecord {count + 1}th episode!")
        events["rerecord_episode"] = False
        continue
      
      # save as hdf5
      h5_dataset.save_to_hdf5(timesteps, actions)
      
      count += 1 
      
