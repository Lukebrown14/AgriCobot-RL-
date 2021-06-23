# Used to contain all the basic functions to control the robot; 

#! /usr/bin/env python

import moveit_commander
from copy import deepcopy
from ur5_vacuum_conveyor.msg import Tracker 
import numpy
import rospy
from openai_ros import robot_gazebo_env_goal
from fetch_moveit_config.fetch_commander import FetchCommander

class RobotEnv(robot_gazebo_env.RobotGazeboEnv):  # !!! RobotGazeboEnv needs creating !!!
	
	def __init__(self):
	
		self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
		self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
		self.cx = 400.0
        	self.cy = 400.0
		self.track_flag = False
		self.phase = 1
		self.Exceeded = False
		self.default_pose_flag = True
		self.justDropped = False 
		
		# Initialize the move group for the ur5_arm
       		 self.arm = moveit_commander.MoveGroupCommander('arm')
        	self.gripper = moveit_commander.MoveGroupCommander("gripper")
		self.fetch_commander_obj = FetchCommander()  # Start the fetch commander object
		
		reference_frame = "/world"#"/base_link"  # Set the reference frame for pose targets
		self.arm.set_pose_reference_frame(reference_frame)  # Set the ur5_arm reference frame accordingly
		
		super(RobotEnv, self).__init__(""" Needs filling """)
		
		# --------- Virtual method ---------
		
	def joint_pose(self, initial_qpos):
	
		self.default_pose = self.arm.get_current_joint_values()
       		self.default_pose[0] = initial_qpos["joint0"]
		self.default_pose[1] = initial_qpos["joint1"]
		self.default_pose[2] = initial_qpos["joint2"]
		self.default_pose[3] = initial_qpos["joint3"]
		self.default_pose[4] = initial_qpos["joint4"]
		self.default_pose[5] = initial_qpos["joint5"]
    
		try:
         		self.arm.set_joint_value_target(self.default_pose)
		 	result = True 
		except Exception as ex:
			print(ex)
			result = False 
	
		return result
		
	def gripper_position(self, action): # Add control of gripper to contrack
	
		self.gripper_pose = self.gripper.get_current_joint_values()
		self.gripper_pose.position.x = action[0]
		self.gripper_pose.position.y = action[1]
		self.gripper_pose.position.z = action[2]
		
		self.gripper.set_joint_value_target(self.gripper_pose)
		
		return True 
	
	def ee_pose(self):
		
		ee_pose = self.fetch_commander_obj.ee_pose()
		return ee_pose
	
	def end_state(self):
		
		self.end_joint_states = deepcopy(self.default_joint_states)
        	self.end_joint_states[0] = -3.65
        
		self.transition_pose = deepcopy(self.default_joint_states)
		self.transition_pose[0] = -3.65
		self.transition_pose[4] = -1.95

		self.gripper.set_joint_value_target( {"Finger1_base_proximal": 0.2})
		self.gripper.go(wait=True)
		
		self.justDropped = True

	def tracking_callback(self, msg): # !!! Needs lookin at !!! 
		
		self.track_flag = msg.flag1
		self.cx = msg.x
		self.cy = msg.y 
		self.error_x = msg.error_x
		self.error_y = msg.error_y
		
		if len(self.pointx) > 9:
			self.track_flag = True 
			
		if self.phase == 2:
			slef.track_flag = False
			self.phase = 1
			
		if self.Exceeded == True and self.error_x < 0:
			self.Exceeded = False 
		
		# Alter -0.6/0.6 to allow RL to change these numbers 
		if self.track_flag == True and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6 and self.Exceeded==False:  #-0.2 in between -0.6 to 0.6 execute to change to a new pose
			self.excute()
			self.defult_pose_flag = False
			
		else:
			if self.waypoints[0].position.x<-0.6:
				self.Exceeded = True 
			if not self.default_pose_flag:
				self.track_flag = False
                self.execute()
				self.default_pose_flag = True

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
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
