# Used to contain all the basic functions to control the robot; 

#! /usr/bin/env python

import moveit_commander
from copy import deepcopy
from ur5_vacuum_conveyor.msg import Tracker  
import rospy
from openai_ros import robot_gazebo_env_goal # This is not right 
from fetch_moveit_config.fetch_commander import FetchCommander

class RobotEnv(robot_gazebo_env.RobotGazeboEnv): 
	
	"""
	!!! RobotGazeboEnv needs creating !!!
	"""
	def __init__(self,track_flag = False,phase = 1,Exceeded = False, model_path ):
		
		if model_path.startswith('/'):
		    fullpath = model_path
		else:
		    fullpath = os.path.join(os.path.dirname(__file__), 'assets', model_path)
		if not os.path.exists(fullpath):
		    raise IOError('File {} does not exist'.format(fullpath))		

		self.track_flag = track_flag 
		self.phase = phase 
		self.Exceeded = Exceeded  
		self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
		self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)

		# Initialize the move group for the ur5_arm
		self.arm = moveit_commander.MoveGroupCommander('arm')
		self.gripper = moveit_commander.MoveGroupCommander('gripper')
		self.fetch_commander_obj = FetchCommander() # Start the fetch commander object
		reference_frame = "/world/base_link"  # Set the reference frame for pose targets
		self.arm.set_pose_reference_frame(reference_frame)  # Set the ur5_arm reference frame accordingly

	
		
		super(RobotEnv, self).__init__(""" Needs filling """)
		
		# --------- Virtual method ---------
		
	def joint_pose(self, initial_qpos): # Setting the position of the joints 
	
		self.initial_qpos = initial_qpos
		
		self.default_pose = self.arm.get_current_joint_values()
		self.default_pose[0] = initial_qpos["joint0"]
		self.default_pose[1] = initial_qpos["joint1"]
		self.default_pose[2] = initial_qpos["joint2"]
		self.default_pose[3] = initial_qpos["joint3"]
		self.default_pose[4] = initial_qpos["joint4"]
		self.default_pose[5] = initial_qpos["joint5"]

		self.arm.set_joint_value_target(self.default_pose)

'''
gripper_position and ee_pose is most likly unneeded 
'''
	def gripper_position(self, initial_grip_pos): # Setting the position of the gripper
	# This is most likly unneeded 
		self.initial_grip_pos = initial_grip_pos 

		self.gripper_pose = self.gripper.get_current_joint_values()
		self.gripper_pose[0] = initial_grip_pos["grip0"]
		self.gripper_pose[1] = initial_grip_pos["grip1"]
		self.gripper_pose[2] = initial_grip_pos["grip2"]
			
		self.gripper.set_joint_value_target(self.gripper_pose)
			
	def ee_pose(self):
		
		ee_pose = self.fetch_commander_obj.ee_pose()
		return ee_pose
	
	def end_state(self): # Repersents what needs to happen after the object is picked up sucessful 
		
		self.end_joint_states = deepcopy(self.default_joint_states)
        	self.end_joint_states[0] = -3.65
        
		self.transition_pose = deepcopy(self.default_joint_states)
		self.transition_pose[0] = -3.65
		self.transition_pose[4] = -1.95

		self.gripper.set_joint_value_target( {"Finger1_base_proximal": 0.2})
		self.gripper.go(wait=True)
		
		self.justDropped = True
		
	def render(self, mode='human', width=DEFAULT_SIZE, height=DEFAULT_SIZE):
		
		self._render_callback()
		if mode == 'rgb_array':
		    self._get_viewer(mode).render(width, height)
		    # window size used for old mujoco-py:
		    data = self._get_viewer(mode).read_pixels(width, height, depth=False)
		    # original image is upside-down, so flip it
		    return data[::-1, :, :]
		elif mode == 'human':
		    self._get_viewer(mode).render()

	def tracking_callback(self, msg): # !!! I'm not sure this correct in tracking the object  
		
		self.track_flag = msg.flag1
		self.cx = msg.x
		self.cy = msg.y 
		self.error_x = msg.error_x
		self.error_y = msg.error_y
		
		if len(self.pointx) > 9:
			self.track_flag = True 
			
		if self.phase == 2:
			self.track_flag = False
			self.phase = 1
			
		if self.Exceeded == True and self.error_x < 0:
			self.Exceeded = False 
		
		# Alter -0.6/0.6 to allow RL to change these numbers 
		if self.track_flag == True and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6 and self.Exceeded == False:  #-0.2 in between -0.6 to 0.6 execute to change to a new pose
			self.excute()
			self.defult_pose_flag = False
			
		else:
			if self.waypoints[0].position.x<-0.6:
				self.Exceeded = True 
			if not self.default_pose_flag:
				self.track_flag = False
				self.default_pose_flag = True 
                self.execute()
		

	# --------- ParticularEnv methods --------- 
	
	def _init_env_variables(self): # Inits variables needed to be initialised each time we reset at the start of an episode.
		
		raise NotImplementedError()
     
	def _compute_reward(self, observations, done): # Calculates the reward to give based on the observations given.
       
	   raise NotImplementedError()

	def execute(self, action): # Applies the given action to the simulation

		raise NotImplementedError()
    
	def _get_obs(self):

		raise NotImplementedError()

	def _is_done(self, observations): # Checks if episode done based on observations given

		raise NotImplementedError()
       
        
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
