#!/usr/bin/python
import rospy
import math
import numpy as np
import mavros_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import mavros
from mavros import command
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from mavros_msgs.msg import State
from std_msgs.msg import String




class base_state():


	def __init__(self):

		self.current_pose = PoseStamped()
		#self.set_pos = PoseStamped()
		self.set_vel = TwistStamped() ## to publish/subscribe to current velocity
		self.current_state = State()
		self.my_state = rospy.Subscriber('/mavros/state',State,self.state_callback)
		self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
		self.local_position_subscribe = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_callback)



		''' update the current state '''
		def state_callback(self,state_data):
			global current_state
			self.current_state = state_data

		self.service_timeout = 30
	    	rospy.loginfo("waiting for ROS services::base state")



class IdleNotArmed(base_state):
	''' Do not do anything with the quadrotor'''
	def run(self):
		pass



#class Arming (base_state):
class Arming (base_state):
	''' This state tries to arm the motors. If they are armed, transition to "Grounded" '''



	def __init__(self):
		super(Arming,self).__init__()

	def run(self):
		# Ensure all services are running, and switch Quad to offboard
		#while current_state.mode != "OFFBOARD" or not current_state.armed:
		#If it is not armed, try to arm, otherwise tell the state machine to switch to armed
		#while not rospy.is_shutdown():
		while self.current_state.mode != "OFFBOARD" or not self.current_state.armed:
			arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	        	arm(True)
	        	set_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
	        	mode = set_mode(custom_mode='OFFBOARD')
	    	else:
	    		self.state_machine_command.publish('Grounded')

	    	rospy.wait_for_service('mavros/set_mode', self.service_timeout)
		rospy.loginfo("ROS services are up :: arming")
		if not mode.mode_sent:
			rospy.logerr("failed to send mode command")
		#rospy.spin()

class Grounded(base_state):
	''' When in this state, the quadrotor is on the ground, or, it is not, it is landing '''

	def run(self):
		while current_state.mode != "AUTO.LAND":
			set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			mode = set_mode(custom='AUTO.LAND')
			rospy.wait_for_service('mavros/set_mode', service_timeout)
			rospy.loginfo("ROS services are up")
		if not mode.mode_sent:
			rospy.logerr("Failed to send mode command")


class Takeoff(base_state):


	'''This state is to take off the quadrotor from ground. if alredy take off wait for command'''
	#print "taking off!!"
	#def __init__(self):
		#pass
	def __init__(self):
		super(Takeoff,self).__init__()

	def pos_sub_callback(self,pose_sub_data):
		global set_pos
		global current_pose
		global pos_pub
		self.current_pose = pose_sub_data
		print "taking off:"
		# Current Position, renamed to shorter variables
		self.x = self.current_pose.pose.position.x
		self.y = self.current_pose.pose.position.y
		self.z = self.current_pose.pose.position.z

		# Goal position
		self.xg = 0
		self.yg = 0
		self.zg = 2

		# Publist to TwistStamped
		self.set_pos.pose.position.x = self.xg
		self.set_pos.pose.position.y = self.yg
		self.set_pos.pose.position.z = self.zg

		self.pos_pub.publish(self.set_pos)





#class Search:





#class Inactive:




#class Landed:






#class Hover:






#class Potential_Avoidance:
