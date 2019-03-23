#!/usr/bin/python
''' Robotics Lab-Boston Univerity '''
"""state_machine.py

This module creates the state_machine node and instantiates all the states, running the active state at 100ms intervals.

Implementing an Inversion of Control structure
To add functionality please add a new state to states.py and modify state change conditions.
"""
import rospy
import states as states_module
import inspect
#from bebop_follow.msg import ChangeState
#from mavros_msgs.msg import State
from std_msgs.msg import String
#from mavros import command
#from std_msgs.msg import String
from quadrotor_state_machine.msg import StateCommand
class StateMachine():
    """State Machine class houses the main state machine.

    Attributes:
        states: A dictionary with string keys of the state class names and State values with each State
        current_state: A string with the name of the active state.
        cmd_vel_topic: A publisher to topic 'cmd_vel_topic'. This publisher controls which topic is being outputted to the bebop
        change_state_topic: A subscriber to topic 'change_state'. This subscriber calls the change_state_wrapper() to change the active state.
    """
    def __init__(self):
        """Initializes StateMachine by instantiating all implemented states and setting GroundedState as the current state

        Creates the state_machine node,
        Create the states dictionary with each class with an implemented next() method in states.py,
        Publishes to cmd_vel_topic and subscribes to change_state
        """
        rospy.init_node('state_machine')
        self.states = {}  #state is a dictionary mapping state names to state objects
        self.current_state = ''
        self.cmd_vel_topic = rospy.Publisher("cmd_vel_topic", String, queue_size=1)
        self.change_state_topic = rospy.Subscriber('state_machine/command', StateCommand, self._change_state_callback)
        for state in inspect.getmembers(states_module,inspect.isclass):                  ##getmembers() function retrieves the members of an object such as a class or module
            if inspect.getmodule(state[1]) == states_module:                             ##state[1] returns the class object of the module named by the file path. This should avoid trying to add classes from nested modules.
                self._add_state(state[1])
        self.change_state('GroundedState', '')
       

    def _add_state(self, new_state_class):
        #import ipdb; ipdb.set_trace()
        # check if this class has a name (i.e., is not a fundamental type), and if there is an object of the same class we are trying to add.
        if hasattr(new_state_class,'__name__') and not new_state_class.__name__ in self.states: 
            new_state_name=new_state_class.__name__
            new_state_object=new_state_class()
            if hasattr(new_state_object, 'flag_add_state') and new_state_object.flag_add_state:
                self.states[new_state_name] = new_state_object
                self.cmd_vel_topic.publish("cmd_vel_" + new_state_name) ##publishing the new state ................. who is subscribing ??
                rospy.loginfo(new_state_name + " added")
            else:
                rospy.logwarn(new_state_name + " not added")
           
    def change_state(self, new_state, caller_state):
        if caller_state == self.current_state or self.current_state == '':
            self._add_state(new_state)
            if type(new_state) == str:
                #if self.new_state.is_transition_allowed(current_state):####think about
                self.current_state = new_state
            else:
                self.current_state = new_state.__name__   ###???????????
            #TODO publish current state on a topic on each call to this method
            #TODO make sure this is the right topic for publishing manual control.
            self.cmd_vel_topic.publish("cmd_vel_" + new_state) 

        
    def _change_state_callback(self, data):
        
       
        self.change_state(data.current, data.next)
       
	

    def run(self):
        """Polls the current state every 100ms to run"""
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state in self.states:
                self.states[self.current_state].run()
            else:
                rospy.logerr("State %s not in dictionary" % self.current_state)
                self._add_state(self.current_state)
            self.rate.sleep()

if __name__ == '__main__':
    sm = StateMachine()
    sm.run()
