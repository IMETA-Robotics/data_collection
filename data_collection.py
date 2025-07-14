from data_collection_cfg import DataCollectionCfg
from imeta_y1_msg.msg import ArmState
from sensor_msgs.msg import Image
import rospy
import os
import collections
import dm_env
import numpy as np
import time
import h5py
# import cv2
from cv_bridge import CvBridge
from control_utils import wait_for_s, is_e_pressed, get_auto_index, start_e_listener

class DataCollection:
  def __init__(self, cfg: DataCollectionCfg):
    self.cfg = cfg
    self.bridge = CvBridge()
    self.master_arm_right_state = None
    self.master_arm_left_state = None
    self.puppet_arm_right_state = None
    self.puppet_arm_left_state = None
    self.img_right = None
    self.img_left = None
    self.img_front = None
    self.img_top = None
    self.depth_right = None
    self.depth_left = None
    self.depth_front = None
    self.depth_top = None
    self.init_topic()
    
  def init_topic(self):
    rospy.init_node("data_collection")
    # subscribe robotic arm data
    rospy.Subscriber(self.cfg.robotif_arm_cfg.master_arm_right_topic,
          ArmState, self.master_arm_right_callback, queue_size=1, tcp_nodelay=True)
    
    rospy.Subscriber(self.cfg.robotif_arm_cfg.puppet_arm_right_topic,
      ArmState, self.puppet_arm_right_callback, queue_size=1, tcp_nodelay=True)
        
    if (self.cfg.robotif_arm_cfg.enable_left_arm):
      rospy.Subscriber(self.cfg.robotif_arm_cfg.master_arm_left_topic,
          ArmState, self.master_arm_left_callback, queue_size=1, tcp_nodelay=True)
      
      rospy.Subscriber(self.cfg.robotif_arm_cfg.puppet_arm_left_topic,
        ArmState, self.puppet_arm_left_callback, queue_size=1, tcp_nodelay=True)
      
    # subscribe camera rgb data
    if self.cfg.camera_cfg.img_right_topic:
      # right arm wrist camera rgb image
      rospy.Subscriber(self.cfg.camera_cfg.img_right_topic, 
        Image, self.img_right_callback, queue_size=1, tcp_nodelay=True)
    
    if self.cfg.camera_cfg.img_left_topic:
      # left arm wrist camera rgb image
      rospy.Subscriber(self.cfg.camera_cfg.img_left_topic, 
        Image, self.img_left_callback, queue_size=1, tcp_nodelay=True)
      
    if self.cfg.camera_cfg.img_front_topic:
      # front external camera rgb image
      rospy.Subscriber(self.cfg.camera_cfg.img_front_topic, 
        Image, self.img_front_callback, queue_size=1, tcp_nodelay=True)
      
    if self.cfg.camera_cfg.img_top_topic:
      # top external camera rgb image
      rospy.Subscriber(self.cfg.camera_cfg.img_top_topic, 
        Image, self.img_top_callback, queue_size=1, tcp_nodelay=True)
      
    # subscribe camera depth data
    if self.cfg.camera_cfg.enable_depth_image:
      if self.cfg.camera_cfg.depth_right_topic:
        # right arm wrist camera depth image
        rospy.Subscriber(self.cfg.camera_cfg.depth_right_topic, 
          Image, self.depth_right_callback, queue_size=1, tcp_nodelay=True)
      
      if self.cfg.camera_cfg.depth_left_topic:
        # left arm wrist camera depth image
        rospy.Subscriber(self.cfg.camera_cfg.depth_left_topic, 
          Image, self.depth_left_callback, queue_size=1, tcp_nodelay=True)
        
      if self.cfg.camera_cfg.depth_front_topic:
        # front external camera depth image
        rospy.Subscriber(self.cfg.camera_cfg.depth_front_topic, 
          Image, self.depth_front_callback, queue_size=1, tcp_nodelay=True)
        
      if self.cfg.camera_cfg.depth_top_topic:
        # top external camera depth image
        rospy.Subscriber(self.cfg.camera_cfg.depth_right_topic, 
          Image, self.depth_right_callback, queue_size=1, tcp_nodelay=True)
      
  def master_arm_right_callback(self, msg: ArmState):
    self.master_arm_right_state = msg 
    
  def puppet_arm_right_callback(self, msg: ArmState):
    self.puppet_arm_right_state = msg
    
  def master_arm_left_callback(self, msg: ArmState):
    self.master_arm_left_state = msg
    
  def puppet_arm_left_callback(self, msg: ArmState):
    self.puppet_arm_left_state = msg
    
  def img_right_callback(self, msg: Image):
    # self.img_right = msg
    self.img_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def img_left_callback(self, msg: Image):
    self.img_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def img_front_callback(self, msg: Image):
    self.img_front = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def img_top_callback(self, msg: Image):
    self.img_top = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_right_callback(self, msg: Image):
    self.depth_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_left_callback(self, msg: Image):
    self.depth_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_front_callback(self, msg: Image):
    self.depth_front = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def depth_top_callback(self, msg: Image):
    self.depth_top = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
  def get_frame(self):
    """Get all the data of a frame"""
    # TODO: 要对数据做时间同步吗？
    if self.master_arm_right_state is None:
      print("master_arm_right_state is None!")
      return False
    
    if self.puppet_arm_right_state is None:
      print("puppet_arm_right_state is None!")
      return False
    
    if self.cfg.robotif_arm_cfg.enable_left_arm:
      if self.master_arm_left_state is None:
        print("master_arm_left_state is None!")
        return False
      
      if self.puppet_arm_left_state is None:
        print("puppet_arm_left_state is None!")
        return False
    
    if self.img_right is None:
      print("img_right is None!")
      return False
    
    return (self.master_arm_right_state, self.puppet_arm_left_state, self.img_right)
    
  def collect(self):
    rate = rospy.Rate(self.cfg.save_rate)
    timesteps = []
    actions = []
    first_step = True
    
    while not rospy.is_shutdown() and not is_e_pressed():
      frame = self.get_frame() 
      if not frame:
        print("get frame failed!")
        rate.sleep()
        continue
      
      # observation
      obs = collections.OrderedDict()
      
      # robotic arm joint state
      if self.cfg.robotif_arm_cfg.enable_left_arm:
        obs["qpos"] = np.concatenate([self.puppet_arm_right_state.joint_position,
                                      self.puppet_arm_left_state.joint_position])
        obs["qvel"] = np.concatenate([self.puppet_arm_right_state.joint_velocity,
                                      self.puppet_arm_left_state.joint_velocity])
        obs["effort"] = np.concatenate([self.puppet_arm_right_state.joint_effort,
                                      self.puppet_arm_left_state.joint_effort])
      else:
        obs["qpos"] = np.array(self.puppet_arm_right_state.joint_position)
        obs["qvel"] = np.array(self.puppet_arm_right_state.joint_velocity)
        obs["effort"] = np.array(self.puppet_arm_right_state.joint_effort)
      
      # camera image data
      image_dict = dict()
      # for camera_name in self.cfg.camera_cfg.camera_names:
      #   image_dict[camera_name] = self.get_image(camera_name)
      image_dict[self.cfg.camera_cfg.camera_names[0]] = self.img_right
      
      obs['images'] = image_dict
      
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
        continue
      
      ts = dm_env.TimeStep(
        step_type=dm_env.StepType.MID,
        reward=None,
        discount=None,
        observation=obs
      )
      # action
      if self.cfg.robotif_arm_cfg.enable_left_arm:
        action = np.concatenate([self.master_arm_right_state.joint_position,
                                 self.master_arm_left_state.joint_position])
      else:
        action = self.master_arm_right_state.joint_position
      
      timesteps.append(ts)
      actions.append(action)
      rate.sleep()
      
    # ctrl-c exit the program  
    if rospy.is_shutdown():
        print("Exit program!")
        exit(-1)
      
    return timesteps, actions
      
  def save_to_hdf5(self, dataset_path, timesteps, actions):
    # extract timesteps and actions to data dict
    len_episode = len(actions)
    if (len_episode == 0): 
      print("Not save, Check this episode data is empty!")
      return
    
    print(f"End collect data! This episode length: {len_episode}. Start save to hdf5 file!")
    data_dict = {
      "/observation/qpos": [],
      "/observation/qvel": [],
      "/observation/effort": [],
      "/action": [],
    }
    for cam_name in self.cfg.camera_cfg.camera_names:
      data_dict["/observation/images/{cam_name}"] = []
    
    while (actions):
      action = actions.pop(0)
      ts = timesteps.pop(0)
      data_dict["/observation/qpos"].append(ts.observation["qpos"])
      data_dict["/observation/qvel"].append(ts.observation["qvel"])
      data_dict["/observation/effort"].append(ts.observation["effort"])
      data_dict["/action"].append(action)
      for came_name in self.cfg.camera_cfg.camera_names:
        data_dict["/observation/images/{cam_name}"].append(ts.observation["images"][cam_name])
        
    # save in hdf5 format
    start_time = time.time()
    with h5py.File(dataset_path, "w") as root:
      root.attrs["sim"] = False
      root.attrs["task_name"] = self.cfg.task_name
      
      # save observation group
      obs = root.create_group("observation")
      obs.create_dataset("qpos", data=np.array(data_dict["/observation/qpos"], dtype=np.float32))
      obs.create_dataset("qvel", data=np.array(data_dict["/observation/qvel"], dtype=np.float32))
      obs.create_dataset("effort", data=np.array(data_dict["/observation/effort"], dtype=np.float32))
      
      # save images group
      images = obs.create_group("images")
      for cam_name in self.cfg.camera_cfg.camera_names:
        images.create_dataset(cam_name, 
                              data=np.array(data_dict["/observation/images/{cam_name}"], 
                              dtype=np.uint8))     
      
      # save action data
      root.create_dataset("action", data=np.array(data_dict["/action"], dtype=np.float32))
      
    print(f"Save episode to {dataset_path}.")
    print(f"Save time: {time.time() - start_time:.3f} secs.")
    
  def run(self):
    if self.cfg.num_episodes < 1:
      raise ValueError("num_episodes must be greater than 0")
    
    dataset_dir = os.path.join(self.cfg.dataset_dir, self.cfg.task_name)
    if self.cfg.start_episode:
      episode_idx = get_auto_index(dataset_dir, self.cfg.start_episode)
    else:
      episode_idx = get_auto_index(dataset_dir)
    
    print("Notice start episode index is : ", episode_idx) 
    
    start_e_listener()
    # 一共可以采集num_episodes条数据
    for i in range(self.cfg.num_episodes):
      # TODO: 每次采集前让机械臂回初始零位？
      
      print("Please press key [S] to start collect {}th episode!".format(i+1))
      wait_for_s()
      
      print("Start collect data! Please press [E] to end collect {}th episode!".format(i+1))
      # collecte one episode
      timesteps, actions = self.collect()
      
      # save hdf5 file
      dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')
      self.save_to_hdf5(dataset_path, timesteps, actions)
      
      episode_idx += 1   