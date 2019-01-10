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
import potential_path_vel as pp
import controller
from bebop_follow.msg import ChangeState
#from mavros_msgs.msg import State

from threading import Timer

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Header
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged

from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged

from geometry_msgs.msg import Twist
#from geometry_msgs.msg import TwistStamped  ## publish or subscribe to velocity 
#from geometry_msgs.msg import PoseStamped        

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry as Odom


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
        self.cmd_vel = rospy.Publisher("cmd_vel_"+type(self).__name__, Twist, queue_size=1)
        #self.cmd_vel = rospy.Publisher("cmd_vel_"+type(self).__name__, TwistStamped, queue_size=1)
        self.msg = Twist()
        #self.msg = TwistStamped()

        rospy.Subscriber("operation_mode", String, self.update_operation_mode) #########???????? 

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

class AutoState(State):
    """AutoState abstract class is for states in autonomous mode.

    Provides Apriltag detection services for tag tracking in autonomous mode.

    Attributes:
        tag_pixel_coordinate: Tuple of ints (x,y) indicating the location of the tag's center on the camera feed
        tag_location: Tuple of ints (x,y,z) indicating the location and orientation of the tag in 3D space relative to the base_link
        tag_found: A boolean indicating if the tag is currently in the camera's vision.
    """
    def __init__(self):
        super(AutoState,self).__init__()
        #Topic to track April Tag
        rospy.Subscriber("tag_pixel_location", Point, self.update_tag_pixel_coordinate)
        rospy.Subscriber("tag_location", Point, self.update_tag_location)
        self.tag_pixel_coordinate = (0,0)
        self.tag_location = (0,0,0)
        rospy.Subscriber("tag_norm", Point, self.update_tag_norm)
        self.tag_norm = (0,0,0)
        rospy.Subscriber("tag_found", Bool, self.update_tag_found)
        self.tag_found = False
        rospy.Subscriber("bebop/odom", Odom, self.update_odom)
        self.angular_velocity = Twist().angular
        self.velocity = Twist().linear
        self.controller_degree = 'first'

    def update_operation_mode(self, data):
        """Handles changes in operation mode on the operator interface

        Args:
            data: A std_msgs/String message.
                data.data must be 'manual' or 'auto' or 'first' or 'second'
        """
        if data.data == 'manual':
            self.next('manual')
        elif data.data == 'auto':
            self.next('auto')
        elif data.data == 'first':
            self.controller_degree = 'first'
            self.next('first')
        elif data.data == 'second':
            self.controller_degree = 'second'
            self.next('second'
        else:
            rospy.logwarn("Unknown mode " + data.data)

    def update_odom(self, data):
        self.angular_velocity = data.twist.twist.angular
        self.velocity = data.twist.twist.linear

    def update_tag_found(self, data):
        if not (self.tag_found == data.data):
            self.tag_found = data.data
            if self.tag_found == True:
                self.next('tag found')
            else:
                self.next('tag lost')

    # Called when the apriltag location is parsed into (x,y) coordinates in pixels
    def update_tag_pixel_coordinate(self, data):
        self.tag_pixel_coordinate = (data.x, data.y)
    
    # Called when the apriltag location is determined relative to the base_link reference frame
    def update_tag_location(self, data):
        self.tag_location = (data.x, data.y, data.z)
    
    # Called when the apriltag normal z vector is determined relative to the base_link reference frame
    def update_tag_norm(self, data):
        self.tag_norm = (data.x, data.y, data.z)
# Search State
# Mode: Follow
# Conditions: Lost April Tag
# Actions: Turn in circles until April Tag is found.
class SearchState(AutoState):
    def __init__(self):
        self.hover = controller.Controller(axis=(0,0))
        self.rotate = controller.Controller(axis=(0.15,))
        super(SearchState, self).__init__()

    def run(self):
        self.msg.linear.x, self.msg.linear.y = self.hover.update()
        self.msg.linear.z = self.msg.angular.x = self.msg.angular.y = 0
        self.msg.angular.z = self.rotate.update()
        super(SearchState, self).run()

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'tag found':
            rospy.loginfo("April Tag found, switching to follow state")
            self.change_state.publish(self.header,'FollowState')
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            self.change_state.publish(self.header,'FlyingState')
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            self.change_state.publish(self.header,'CriticalState')
        else:
            pass

# Velocity State
# Mode: Follow
# Conditions: April Tag in sight
# Actions: Keep April Tag at the center of the camera && Keep bebop a fixed distance away from the robot
class VelocityState(AutoState):
    def __init__(self):
        self.yaw_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(428,240), z=0.0015, z_v=0.3, kind='hybrid', d_threshold=150),
                obstacles=(pp.ObstaclePotential(location=(428,240), kind='circle', d_safe=100, r=300, n=600, n_v=0.1, keep_out='in', max_gradient=0.3),)
                )
        #self.yaw_follow = controller.Controller(axis=(0,0))
        
        self.longitudinal_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(-2,0), z=0.2, z_v=0.2, k_i= 0.00, kind='hybrid', d_threshold=0.3),
                obstacles=()
                )
        
        self.lateral_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(0,0), z=0.2, z_v=0.2,k_i=0.0, kind='hybrid', d_threshold=0.3),
                obstacles=()
                )
        self.lateral_orient = controller.PotentialController(
                goal=pp.GoalPotential(location=(0,0), z=0.1, z_v=0.2, kind='hybrid', d_threshold=0.2),
                obstacles=()
                )
        #self.longitudinal_follow = controller.Controller(axis=(0,0))
        #self.lateral_follow = controller.Controller(axis=(0,0))
        super(VelocityState, self).__init__()

    def run(self):
        if not self.controller_degree == 'second':
            self.next(self.controller_degree)
        self.msg.linear.z = self.msg.angular.x = self.msg.angular.y = 0
        self.msg.linear.x = self.longitudinal_follow.update(location=(-self.tag_location[0], 0), velocity=(self.velocity.x,0))[0]
        self.msg.linear.y = self.lateral_follow.update(location=(self.tag_norm[1],0), velocity=(self.velocity.y,0))[0]
        self.msg.linear.y = self.msg.linear.y + self.lateral_orient.update(location=(self.tag_location[1], 0), velocity=(self.velocity.y,0))[0]
        self.msg.angular.z = self.yaw_follow.update(location=self.tag_pixel_coordinate, velocity=self.angular_velocity.z)[0]
        #self.msg.linear.y= 0
        super(VelocityState, self).run()
        


    def tag_lost(self):
        if not self.tag_found:
            rospy.loginfo("April Tag lost, switching to search state")
            self.change_state.publish(self.header,'SearchState')
        else:
            rospy.loginfo("~~April Tag recovered~~")

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'tag lost':
            rospy.loginfo("April Tag lost, waiting for confirmation")
            Timer(0.5,self.tag_lost).start()
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            self.change_state.publish(self.header,'FlyingState')
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            self.change_state.publish(self.header,'CriticalState')
        elif event == 'first':
            rospy.loginfo("Switching to First Degree Controller")
            self.change_state.publish(self.header,'FollowState')
        else:
            pass

# Follow State
# Mode: Follow
# Conditions: April Tag in sight
# Actions: Keep April Tag at the center of the camera && Keep bebop a fixed distance away from the robot
class FollowState(AutoState):
    def __init__(self):
        #self.longitudinal_follow = controller.Controller(axis = (0,0))
        
        self.longitudinal_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(-2,0), z=0.1, kind='hybrid', d_threshold=0.3),
                obstacles=()
                )
        self.lateral_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(0,0), z=0.1, kind='hybrid', d_threshold=0.3),
                obstacles=()
                )
        self.yaw_follow = controller.PotentialController(
                goal=pp.GoalPotential(location=(428,240), z=0.0015, kind='hybrid', d_threshold=150),
                obstacles=(pp.ObstaclePotential(location=(428,0), kind='circle', d_safe=100, r=300, n=600, keep_out='in', max_gradient=0.3),)
                )
        super(FollowState, self).__init__()

    def run(self):
        if not self.controller_degree == 'first':
            self.next(self.controller_degree)
        self.msg.linear.x = self.longitudinal_follow.update(((-self.tag_location[0], 0)))[0]
        self.msg.linear.y = self.lateral_follow.update((self.tag_norm[1],0))[0]
        self.msg.linear.z = self.msg.angular.x = self.msg.angular.y = 0
        self.msg.angular.z = self.yaw_follow.update((self.tag_pixel_coordinate[0],0))[0]
        super(FollowState, self).run()

    def tag_lost(self):
        if not self.tag_found:
            rospy.loginfo("April Tag lost, switching to search state")
            self.change_state.publish(self.header,'SearchState')
        else:
            rospy.loginfo("~~April Tag recovered~~")

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'tag lost':
            rospy.loginfo("April Tag lost, waiting for confirmation")
            Timer(0.5,self.tag_lost).start()
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            self.change_state.publish(self.header,'FlyingState')
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            self.change_state.publish(self.header,'CriticalState')
        elif event == 'second':
            rospy.loginfo("Switching to Second Degree Controller")
            self.change_state.publish(self.header,'VelocityState')
        else:
            pass

# Critical State
# Mode: Follow, Manual
# Conditions: Low Battery
# Actions: Land
class CriticalState(State):
    def __init__(self):
        rospy.logwarn("Landing")
        self.land = rospy.Publisher("land", Empty, queue_size=1)
        super(CriticalState, self).__init__()

    def run(self):
        super(CriticalState, self).run()
        self.land.publish()

    def next(self, event):
        self.header.stamp = rospy.Time.now()
        if event == 'state_machine_ready':
            return True
        elif event == 'grounded':
            rospy.loginfo("Successfully landed, switching to Grounded State")
            self.change_state.publish(self.header,'GroundedState')
        elif event == 'low battery':
            self.land.publish() #TODO consider switching to emergency landing when low battery is warned a second time.
            rospy.logwarn("Second Low Battery")
            self.change_state.publish(self.header,type(self).__name__)
        else:
            self.change_state.publish(self.header,type(self).__name__)

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
