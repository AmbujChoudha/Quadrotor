#!/usr/bin/python
''' Robotics Lab-Boston Univerity '''
""" states.py
2 Big States w/ sub states
- Auto Mode
-- AprilTag Lost -> Spin around
-- Following April Tag
-- Low on Battery -> switching to Manual Mode
- Manual Mode
-- Landed -> wait for takeoff command. Disable all other movements
-- In air

Each state holds a controller of their own and publishes commands to the cmd_pub_m.py through the topic cmd_vel_[state name]

To be used in conjucntion with operator.py and potential_path.py
"""
import rospy
import mavros_msgs.msg
import math
import numpy as np
from threading import Timer
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Header
#from mavros_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped  ## publish or subscribe to velocity 
from geometry_msgs.msg import PoseStamped        
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry as Odom
from mavros_msgs.srv import CommandBool, ParamGet, SetMode


##important ros msg 
##rosmsg show sensor_msgs/BatteryState
##rostopic info /mavros/battery

class State(object):
    """State class is the base class 

    Attributes:
        cmd_vel: A Publisher that publishes velocity commands to cmd_pub_m. The topic published is 'cmd_vel_[Class Name]'
        msg: A Twist message to be published to cmd_vel
        header: A Header message to be published to change_state
        change_state: A Publisher that publishes to the state machine about changes in the desired state.
    """
    def __init__(self):
        """Initializes the State.

        Creates publishers cmd_vel and change_state
        Subscribes to operation_mode, FlyingStateChanged, and AlertStateChanged
        Sets the header's frame_id to the class name
        """
        
        self.flag_add_state=True  
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
        #self.msg = Twist()
        self.msg = TwistStamped()
        self.current_pose = PoseStamped()
        self.current_state = mavros_msgs.msg.State()
        self.local_position_subscribe = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.update_local_position)
        self.my_state = rospy.Subscriber('/mavros/state',mavros_msgs.msg.State,self.state_callback)
        #rospy.Subscriber("bebop/states/ardron3/PilotingState/AlertStateChanged", Ardrone3PilotingStateAlertStateChanged, self.update_alert)
        #rospy.Subscriber("/mavros/battery", BatteryState, self.update_alert)


        #TODO invesitgate if it would be a problem if all the states change to critical state and the active state's message gets pushed out of the buffer.
        #self.change_state = rospy.Publisher("change_state", ChangeState, queue_size = 10)
        self.service_timeout = 30 
        rospy.loginfo("waiting for ROS services::base state")

    def update_local_position(self, data):
        ''' update the quadrotor local position ''' 
        self. current_pose = data 
        self.x = self.current_pose.pose.position.x 
        self.y = self.current_pose.pose.position.y 
        self.z = self.current_pose.pose.position.z 


    def state_callback (self, state_data):
        """ updating the current state """ 
        global current_state 
        self.current_state = state_data


    def run(self):
        self.cmd_vel.publish(self.msg)

'''
# Flying State
# Mode: Follow
# Conditions: Motors on with operator control
# Actions: Obey Operator Control
class FlyingState(State):
    def __init__(self):
        super(FlyingState,self).__init__()

    def run(self):
        #TODO remap operator controls over to flying state?
        pass

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'grounded':
            rospy.loginfo("Landed, switching to grounded state")
            self.change_state.publish(self.header,'GroundedState')
        elif event == 'auto':
            rospy.loginfo("Entering Autonomous mode, switching to search state")
            self.change_state.publish(self.header,'SearchState')
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            self.change_state.publish(self.header,'CriticalState')
        else:
            pass

class FlyingState(State):

    def __init__(self):
        super(FlyingState, self).__init() 

    def run (self):
        #gola position 
        xg = 0 
        yg = 0
        zg = 4 
    # Position error between setpoint and current position 
    x_error = xg - x 
    y_error = yg - y 
    z_error = zg - z 

    #velocity vector to get to the goal 
    gx = .5*x_error 
    gy = .5*y_error
    gz = .7*z_error

    #set limits on the velocity the quad can have 
    if abs(gx) > 2: 
        gx = np.sign(gx)*2 
    if abs(gy) > 2: 
        gy = np.sign(gy)*2
    if abs(gz) > 2: 
        cz = np.sign(gz)*2


    self.msg.linear.x = gx 
    self.msg.linear.y = gy 
    self.msg.linear.z = gz 

    super(FlyingState, self).run()


'''
'''
# Grounded State
# Mode: Follow, Manual
# Conditions: Motors off
# Actions: Nothing
class GroundedState(State):
    def __init__(self):
        super(GroundedState, self).__init__()

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'flying':
            rospy.loginfo("Successfully took off, switching to Flying State")
            self.change_state.publish(self.header,'FlyingState')
        else:
            pass
'''

class GroundedState(State):
    def __init__(self):
        State.__init__(self) 
        self.flag_add_state=True
#self.service_timeout = 30
#rospy.loginfo("waiting for ROS services")
    def run(self):
        #import ipdb; ipdb.set_trace()
        while self.current_state != "AUTO.LAND": 
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            mode = set_mode (custom_mode = 'AUTO.LAND')
            rospy.wait_for_service('mavros/set_mode',self.service_timeout)
        if not mode.mode_sent: 
            rospy.logerr("failed to send mode command")

    def is_transition_allowed(self,new_state):
        return new_state in ['ArmingState']###wh
        return new_state not in ['Flying']####bl


class ArmingState (State):
    ''' This state will arm the quadrotor ''' 
    def __init__(self):
        super(ArmingState, self).__init__()
        self.flag_add_state=True  

    def run(self):
        ''' Ensure all services are running, and switch Quad to offboard '''
        while self.current_state != "OFFBOARD" or not self.current_state.armed: 
            arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            arm(True)
            #set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            #mode = set_mode(custom_mode = 'OFFBOARD')
            #rospy.wait_for_services('mavros/set_mode', self.service_timeout)
        rospy.loginfo("ROS services are up::arming")
        #if not mode.mode_sent: 
         #   rospy.logerr("falied to send mode command")

