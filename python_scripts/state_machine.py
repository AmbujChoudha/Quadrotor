#!/usr/bin/python
"""state_machine.py
State Machine for the Bebop Follow package.

This module creates the state_machine node and instantiates all the states, running the active state at 100ms intervals.

Implementing an Inversion of Control structure
To add functionality please add a new state to states.py and modify state change conditions.
"""
import rospy
import states
import inspect
from bebop_follow.msg import ChangeState
#from mavros_msgs.msg import State
from std_msgs.msg import String

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
        self.states = {}
        self.current_state = ''
        self.cmd_vel_topic = rospy.Publisher("cmd_vel_topic", String, queue_size=1)
        self.change_state_topic = rospy.Subscriber("change_state", ChangeState, self._change_state_wrapper) 
        for state in inspect.getmembers(states,inspect.isclass):                  ##getmembers() function retrieves the members of an object such as a class or module  
            if inspect.getmodule(state[1]) == states:                             ##Return the name of the module named by the file path
                self._add_state(state[1])
        self.change_state('GroundedState', '')

    def _add_state(self, new_state):
        """Adds a state to the dictionary
        
        The string name of the state is used as the key for the dictionary and the value is an instance of the state
        Ensures that the class has a callable next method. If there is one, the program assumes that the class is properly implemented and not an abstract state.

        Args:
            new_state: A string of the class name for the particular state or a class that extends State
        """
        if type(new_state) == type:             ##?????????????????????
            new_state = new_state.__name__      ##??????????????/
        if not new_state in self.states:
            new_state_instance = getattr(states, new_state) 
            try:
                if callable(new_state_instance.next):   ##what is callable next method ????????????????????/
                    self.states[new_state] = new_state_instance()
                    self.cmd_vel_topic.publish("cmd_vel_" + new_state) ##publishing the new state ................. who is subscribing ??
                else:
                    raise NotImplementedError("method next is not callable")
            except:
                rospy.logwarn(new_state + " not added")

    def change_state(self, new_state, caller_state):
        """Changes the state to new_state

        Only works if caller_state is the current active state.
        If the new_state is not currently in the states dictionary, adds the new_state to the dictionary.
        Sets the new_state to be the active state and allows cmd_pub_m to read from it.

        Args:
            new_state: A string with the name of the class of the desired new state. 
                Or a derivative of the State class.
            caller_state: A string with the name of the class of the state that called the change_state function.
        """
        if caller_state == self.current_state or self.current_state == '':
            self._add_state(new_state)
            if type(new_state) == str:
                self.current_state = new_state
            else:
                self.current_state = new_state.__name__   ###???????????
            #TODO make sure this is the right topic for publishing manual control.
            self.cmd_vel_topic.publish("cmd_vel_" + new_state) 

    def _change_state_wrapper(self, data):
        """A wrapper method for change_state

        Converts ChangeState message into a format that change_state() can read, separating the new_state and the caller state

        Args:
            data: A ChangeState message.
        """
        self.change_state(data.state, data.header.frame_id)

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
