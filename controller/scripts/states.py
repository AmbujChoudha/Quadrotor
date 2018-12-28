
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


current_pose = PoseStamped()
set_vel = TwistStamped()
current_state = State()

def state_callback(state_data):
	global current_state
	current_state = state_data

class base_state():

	def __init__(self):
		self.my_state = rospy.Subscriber('/mavros/state',State,state_callback) #subscribing to local state
		self.service_timeout = 30
	    	rospy.loginfo("waiting for ROS services::base state")



class IdleNotArmed(base_state):
	''' Do not do anything with the quadrotor'''
	def run(self):
		pass



#class Arming (base_state):
class Arming (base_state):
	''' This state tries to arm the motors. If they are armed, transition to "Grounded" '''



	#def __init__(self):
		#super().__init__(self):

	def run(self):
		# Ensure all services are running, and switch Quad to offboard
		#while current_state.mode != "OFFBOARD" or not current_state.armed:
		#If it is not armed, try to arm, otherwise tell the state machine to switch to armed
		while not rospy.is_shutdown():
			if current_state.mode != "OFFBOARD" or not current_state.armed:
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
		rospy.spin()

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

	def __init__(self):

		print "taking off!!"
		global vel_pub
		# Set up publishers and subscribers
		self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_callback)
		# Set the timeout for the ROS service checks
		service_timeout = 30
		rospy.loginfo("waiting for ROS services::takeoff")

		# Keep program alive until we stop it
		rospy.spin()

	def pos_sub_callback(self,pose_sub_data):
		##global set_val
		global current_pose
		global vel_pub
		current_pose = pose_sub_data

		# Current Position, renamed to shorter variables
		x = current_pose.pose.position.x
		y = current_pose.pose.position.y
		z = current_pose.pose.position.z

		# Goal position
		xg = 2
		yg = 2
		zg = 2

		# Position error between setpoint and current position
		x_error = xg - x
		y_error = yg - y
		z_error = zg - z

		# Publist to TwistStamped
		set_vel.twist.linear.x = .5*x_error
		set_vel.twist.linear.y = .5*y_error
		set_vel.twist.linear.z = .7*z_error

		if abs(set_vel.twist.linear.x) > 2:
			set_vel.twist.linear.x = np.sign(set_vel.twist.linear.x)*2
		if abs(set_vel.twist.linear.y) > 2:
	        	set_vel.twist.linear.y = np.sign(set_vel.twist.linear.y)*2
		if abs(set_vel.twist.linear.z) > 2:
	        	set_vel.twist.linear.z = np.sign(set_vel.twist.linear.z)*2

		self.vel_pub.publish(set_vel)





#class Search:





#class Inactive:




#class Landed:






#class Hover:






#class Potential_Avoidance:
