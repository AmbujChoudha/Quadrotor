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
from threading import Timer


class State(object):
    """State class is the base class for the bebop_follow state machine

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
        self.cmd_vel = rospy.Publisher("cmd_vel_"+type(self).__name__, TwistStamped, queue_size=1)
        self.msg = TwistStamped()
        rospy.Subscriber("operation_mode", String, self.update_operation_mode)
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.update_flying)
        rospy.Subscriber("bebop/states/ardron3/PilotingState/AlertStateChanged", Ardrone3PilotingStateAlertStateChanged, self.update_alert)
        #TODO invesitgate if it would be a problem if all the states change to critical state and the active state's message gets pushed out of the buffer.
        self.header = Header()
        self.her.frame_id = type(self).__name__
        self.change_state = rospy.Publisher("change_state", ChangeState, queue_size = 10)

    def update_operation_mode(self, data):
        """Handles changes in operation mode on the operator interface

        Args:
            data: A std_msgs/String message.
                data.data must be 'manual' or 'auto'
        """
        if data.data == 'manual':
            self.next('manual')
        elif data.data == 'auto':
            self.next('auto')
        else:
            rospy.logwarn("Unknown mode " + data.data)

    def update_alert(self, data):
        """Handles alerts given by the bebop. 
        
        Args:
            data: An Alert State Changed message.
        """
        if data.state == data.state_low_battery or data.state == data.state_critical_battery:
            self.next('low battery')
        else:
            rospy.logwarn("Other warning: " + str(data.state))

    def update_flying(self, data):
        """Handles alerts about the Flying State changing

        Alerts the states as to when the Bebop if flying and when it has landed.

        Args:
            data: A FlyingState message from the bebop.
        """
        if data.state == data.state_landed:
            self.next("grounded")
        elif data.state == data.state_hovering or data.state == data.state_flying:
            self.next("flying")
        elif data.state == data.state_emergency_landing:
            rospy.logwarn("Emergency Landing State Entered")

    def run(self):
        self.cmd_vel.publish(self.msg)



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


# Arming State
# Mode: arming, auto
# Conditions: Motors on
# Actions: motor spinning
