from utils.data_collection_cfg import DataCollectionCfg
from imeta_y1_msg.msg import ArmJointState
from sensor_msgs.msg import Image
import rospy
import collections
import dm_env
import numpy as np
# import cv2
from cv_bridge import CvBridge
from utils.data_handle import wait_for_s, is_e_pressed, start_e_listener
from dataset.hdf5_dataset import H5Dataset
from dataset.lerobot_dataset import LRDataset

class DataCollection:
  def __init__(self, cfg: DataCollectionCfg):
    self.cfg = cfg
    self.bridge = CvBridge()
    self.master_arm_right_state = None
    self.master_arm_left_state = None
    self.puppet_arm_right_state = None
    self.puppet_arm_left_state = None
    self.img_dict = {}
    # self.depth_right = None
    # self.depth_left = None
    # self.depth_front = None
    # self.depth_top = None
    self.depth_dict = {}
    self.init_topic()
    
  def init_topic(self):
    rospy.init_node("data_collection")
    # subscribe robotic arm data
    rospy.Subscriber(self.cfg.robotif_arm_cfg.master_arm_right_topic,
          ArmJointState, self.master_arm_right_callback, queue_size=1, tcp_nodelay=True)
    
    rospy.Subscriber(self.cfg.robotif_arm_cfg.puppet_arm_right_topic,
      ArmJointState, self.puppet_arm_right_callback, queue_size=1, tcp_nodelay=True)
        
    if (self.cfg.robotif_arm_cfg.enable_left_arm):
      rospy.Subscriber(self.cfg.robotif_arm_cfg.master_arm_left_topic,
          ArmJointState, self.master_arm_left_callback, queue_size=1, tcp_nodelay=True)
      
      rospy.Subscriber(self.cfg.robotif_arm_cfg.puppet_arm_left_topic,
        ArmJointState, self.puppet_arm_left_callback, queue_size=1, tcp_nodelay=True)
      
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
      
  def master_arm_right_callback(self, msg: ArmJointState):
    self.master_arm_right_state = msg 
    
  def puppet_arm_right_callback(self, msg: ArmJointState):
    self.puppet_arm_right_state = msg
    
  def master_arm_left_callback(self, msg: ArmJointState):
    self.master_arm_left_state = msg
    
  def puppet_arm_left_callback(self, msg: ArmJointState):
    self.puppet_arm_left_state = msg
    
  def img_right_callback(self, msg: Image):
    self.img_dict["cam_right_wrist"] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
  def img_left_callback(self, msg: Image):
    self.img_dict["cam_left_wrist"] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
  def img_front_callback(self, msg: Image):
    self.img_dict["cam_front"] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
  def img_top_callback(self, msg: Image):
    self.img_dict["cam_top"] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
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
    frame = {}
    
    # TODO: 要对数据做时间同步吗？
    if self.master_arm_right_state is None:
      print("master_arm_right_state is None!")
      return False
    
    if self.puppet_arm_right_state is None:
      print("puppet_arm_right_state is None!")
      return False
    
    if self.cfg.robotif_arm_cfg.enable_left_arm:
      # 双臂摇操
      if self.master_arm_left_state is None:
        print("master_arm_left_state is None!")
        return False
      
      if self.puppet_arm_left_state is None:
        print("puppet_arm_left_state is None!")
        return False
      
      frame["state"] = np.concatenate([self.puppet_arm_right_state.joint_position,
                                      self.puppet_arm_left_state.joint_position])
      frame["action"] = np.concatenate([self.master_arm_right_state.joint_position,
                                 self.master_arm_left_state.joint_position])
      if self.cfg.robotif_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.concatenate([self.puppet_arm_right_state.joint_velocity,
                                              self.puppet_arm_left_state.joint_velocity])
      if self.cfg.robotif_arm_cfg.use_joint_effort:
        frame["effort"] = np.concatenate([self.puppet_arm_right_state.joint_effort,
                                          self.puppet_arm_left_state.joint_effort])
      
    else:
      # 单臂摇操，默认为右臂
      frame["state"] = np.array(self.puppet_arm_right_state.joint_position)
      frame["action"] = self.master_arm_right_state.joint_position
      if self.cfg.robotif_arm_cfg.use_joint_velocity:
        frame["velocity"] = np.array(self.puppet_arm_right_state.joint_velocity)
      if self.cfg.robotif_arm_cfg.use_joint_effort:
        frame["effort"] = np.array(self.puppet_arm_right_state.joint_effort)
    
    cameras = {}
    for cam_name in self.cfg.camera_cfg.camera_names:
      if cam_name == "cam_right_wrist" and self.img_dict["cam_right_wrist"] is None:
        print("right image is None!")
        return False
      if cam_name == "cam_left_wrist" and self.img_dict["cam_left_wrist"] is None:
        print("left image is None!")
        return False
      if cam_name == "cam_front" and self.img_dict["cam_front"] is None:
        print("front image is None!")
        return False
      if cam_name == "cam_top" and self.img_dict["cam_top"] is None:
        print("top image is None!")
        return False
      
      cameras[cam_name] = self.img_dict[cam_name]
      
    frame["cameras"] = cameras
     
    return frame
    
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
      
      # state
      obs["state"] = frame["state"]

      if self.cfg.robotif_arm_cfg.use_joint_velocity:
        obs["velocity"] = frame["velocity"]
      if self.cfg.robotif_arm_cfg.use_joint_effort:
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
        continue
      
      ts = dm_env.TimeStep(
        step_type=dm_env.StepType.MID,
        reward=None,
        discount=None,
        observation=obs
      )
      
      timesteps.append(ts)
      actions.append(frame["action"])
      rate.sleep()
      
    # ctrl-c exit the program  
    if rospy.is_shutdown():
        print("Exit program!")
        exit(-1)
      
    return timesteps, actions
    
  def run(self):
    if self.cfg.num_episodes < 1:
      raise ValueError("num_episodes must be greater than 0")
    
    # whether save data as hdf5
    if self.cfg.hdf5_cfg.save_as_h5:
      h5_dataset = H5Dataset(self.cfg)
    
    # whether save data as lerobot dataset
    if self.cfg.lerobot_dataset_cfg.save_as_lerobot:
      lerobot_dataset = LRDataset(self.cfg)
    
    start_e_listener()
    # 一共可以采集num_episodes条数据
    for i in range(self.cfg.num_episodes):
      # TODO: 每次采集前让机械臂回初始零位？
      print("Please press key [S] to start collect {}th episode!".format(i+1))
      wait_for_s()
      
      print("Start collect data! Please press [E] to end collect {}th episode!".format(i+1))
      # collecte one episode
      timesteps, actions = self.collect()
      
      # save as hdf5
      if self.cfg.hdf5_cfg.save_as_h5:
        h5_dataset.save_to_hdf5(timesteps, actions)
        
      # save as lerobotdatset
      if self.cfg.lerobot_dataset_cfg.save_as_lerobot:
        lerobot_dataset.save_to_lerobot(timesteps, actions)