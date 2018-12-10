#/usr/bin/python
#Reference: PX4/Firmware 
import rospy
import math
import numpy as np
import mavros_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import mavros
from mavros import command
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from mavros_msgs.msg import State



current_pose = PoseStamped()   
set_vel = TwistStamped()
current_state = State()


def state_callback(state_data):
	global current_state
	current_state = state_data

def main():
	global vel_pub
	rospy.init_node('Velocity_Control', anonymous='True')

	my_state = rospy.Subscriber('/mavros/state',State,state_callback) #subscribing to local state 
	vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1) #publishing the velocity 
	#local_position_subscribe = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pos_sub_callback) #updating the local position

	service_timeout = 30
        rospy.loginfo("waiting for ROS services")

	while current_state.mode != "OFFBOARD" or not current_state.armed:
        	arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        	arm(True)
        	set_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        	mode = set_mode(custom_mode='OFFBOARD')
		rospy.wait_for_service('mavros/set_mode', service_timeout)
		rospy.loginfo("ROS services are up")
		if not mode.mode_sent:
			rospy.logerr("failed to send mode command")


	rospy.spin()

if __name__ == "__main__":
    main()
