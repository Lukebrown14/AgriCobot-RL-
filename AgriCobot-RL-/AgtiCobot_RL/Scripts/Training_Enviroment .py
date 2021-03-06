
from AgtiCobot_RL.Scripts import RobotEnv, utils 
from gym.envs.registration import register
import rospy
import numpy as np 
import math 
from object_position import Obj_Pos  

timestep_limit_per_episode = 100

register(
        id='TrainingEnv-v0',
        entry_point='Training_Enviroment:trainingEnv',  
        timestep_limit=timestep_limit_per_episode,
    )

class TrainingEnv(Robot_Enviroment.RobotEnv, utils.EzPickle): 

    def __init__(self):
		
        self.parameters()
        Robot_Enviroment.RobotEnv.__init__(self)
        utils.EZPickle.__init__(self) 
		
        self.gazebo.unpauseSim()
		
	        # self.action_space = spaces.Discrete(self.n_actions)
        self.action_space = self.spaces.Box(
		    low = self.position_joints_min,
		    high = self.position_joints_max, shape=(self.n_actions,),
		    dtype = np.float32)
        
        observations_high_dist = np.array([self.max_distance])
        observations_low_dist = np.array([0.0])

        observations_high_speed = np.array([self.max_speed])
        observations_low_speed = np.array([0.0])

        observations_ee_z_max = np.array([self.ee_z_max])
        observations_ee_z_min = np.array([self.ee_z_min])

        high = np.concatenate([observations_high_dist, observations_high_speed, observations_ee_z_max])
        low = np.concatenate([observations_low_dist, observations_low_speed, observations_ee_z_min])

        self.observation_space = self.spaces.Box(low, high)

        obs = self._get_obs()
		
	def parameters(self):
		
         self.reached_goal_reward = 10
        self.impossible_movement_punishememt = -10 
        self.closer_reward = 1
        self.step_punishment = -1 
        self.ee_z_min = 0.3  # Normal z pos of cube minus its height/2
        self.ee_z_max = 1.0
		
        self.sim_time = rospy.get_time()
        self.has_object = False
        self.failed = False 
		
        self.initial_qpos = {"joint0": -1.57691,
		"joint1": -1.71667,
		"joint2": 1.79266,
		"joint3": -1.67721,
		"joint4": -1.5705,
		"joint5": 0.0,}
		
    def _set_init_pose(self): # Sets the Robot in its init pose

        self.gazebo.unpauseSim()
        if not self.joint_pose (self.initial_qpos):
           assert False, "Initialisation is failed...."

    def _init_env_variables(self): # Inits variables needed to be initialised each time we reset at the start of an episode
		
        rospy.logdebug('Init Env Variables...')
        rospy.logdebug('Init Env Variables... End') 

    def execute(self, action): # Move the robot based on the action variable given
	
        if self.track_flag == True:	 

            self.new_pos = {"joint0": action[0],  # How does action relate to everything 
				"joint1": action[1],
				"joint2": action[2],
				"joint3": action[3],
				"joint4": action[4],
				"joint5": action[5],
				"joint6": action[6]}

            self.new_grip_pose = {}

            self.movement_result = self.joint_pose(self.new_pos)
            self.gripper.set_joint_value_target( {"Finger1_base_proximal": action[7]}) # !!! Needs working on !!! was 1  
            self.gripper.go()
            has_object = True 

            if has_object == True:
               self.end_state() # This is meant to excute drop of object in Robot_Enviroment 
               return self.task_completed 	
            else:
               self.missed  

            final_outcome = self.task_completed or self.missed 
				
        return final_outcome
		
    def _get_obs(self): # Here we define what sensor data of our robots observations To know which Variables we have acces to

        self.gazebo.unpauseSim()
        grip_pose = self.ee_pose()
        gripper_array = [grip_pose.position.x, grip_pose.position.y, grip_pose.position.z] 
		
		# Pose of object         
        object_data = self.obj_positions.get_states()
        object_pos = object_data[3:]
		
		# Distance from object 
        distance_from_cube = self.calc_dist(object_pos,gripper_array)
		
		# object detection
        object_vision = self.tracking_callback()
        vision_array = [object_vision.cx, object_vision.cy]
		
        observations = np.array([distance_from_cube,gripper_array, vision_array])

        return observations
		
    def calc_dist(self,p1,p2): # Calculates the distnace between the object and gripper 

        x_d = math.pow(p1[0] - p2[0],2)
        y_d = math.pow(p1[1] - p2[1],2)
        z_d = math.pow(p1[2] - p2[2],2)
        d = math.sqrt(x_d + y_d + z_d)

        return d
		
    def get_elapsed_time(self): # Returns the elapsed time since the beginning of the simulation
		
        current_time = rospy.get_time()
        dt = self.sim_time - current_time
        self.sim_time = current_time

        return dt

    def _is_done(self, execute): # Decide if episode is done based on the observations
	
        if self.missed == True:
            self.done_fail
		
        if self.task_completed == True:
            self.done_sucess 
			
        if self.dt == 100: # Needs work / When the time of the system reach 100 seconds it timesout and finishes  
          return self.timeout 
			
        done = self.done_fail or self.done_sucess or self.timeout 
	   
        return done

    def _compute_reward(self, observations, done): # Return the reward based on the observations given
		
        distance = observations[0]
        ee_z_pos = observations[2]
		
        if self.done_fail:
            reward = self.impossible_movement_punishememt # If the task fails 
        else :
            if self.done_sucess:
               reward = self.reached_goal_reward # if it pick and places the object 
            if ee_z_pos < self.ee_z_min or ee_z_pos >= self.ee_z_max:
                    reward = self.impossible_movement_punishememt / 4.0 
            else:
                reward = 1.0 / distance  # How close it got the object 
		
		for x in self.dt:  # Needs work / Every 10 sec it lose a -1 from the reward
             if x % 10 == 0:
                reward -= self.step_punishment
				
		return reward
				
        
    # Internal TaskEnv Methods

