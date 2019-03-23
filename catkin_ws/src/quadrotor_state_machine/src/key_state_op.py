#!/usr/bin/env python
#import state_machine.py
import rospy
#import math
#import numpy as np
#import mavros_msgs
#from geometry_msgs.msg import PoseStamped, TwistStamped
#import mavros
#from mavros import command
#from mavros_msgs.srv import CommandBool, ParamGet, SetMode
#from mavros_msgs.msg import State
#import states
#import stateMachine.py
#from std_msgs.msg import String
from quadrotor_state_machine.msg import StateCommand



class Key_State_Op():
    def menu(self):
        ##FlyingState
        print ("Press")
        print ("A: to set mode to ARM the drone")
        print ("S: to set mode to STABILIZE")
        print ("D: to set mode to DISARM the drone")
        print ("T: to set mode to TAKEOFF")
        print ("G: to set mode to GroundState")
        print ("F: to set mode to FlyingState")
        print ("L: to set mode to LAND")
        print ("LO: print GPS coordinates")
        print ("E: to exit")

    def publish(self, str):
        print ('state_machine/command <- ', str)
        self.state_machine_command.publish(str);

    def __init__(self):
        self.state_machine_command=rospy.Publisher('state_machine/command',StateCommand,queue_size = 1) ##publishing the string msg

    def run(self):
        self.msg=StateCommand()
        self.menu()
        x='1'
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            x = raw_input ("Enter your input: ")
            self.msg.current=''  ###current is not being updated ?????
            print ("|",x,"|")
            if x=='A':
                self.msg.next='ArmingState'
                self.state_machine_command.publish(self.msg)
            elif x == 'G':
                self.msg.next = 'GroundedState'
                self.state_machine_command.publish(self.msg)
            elif x == 'F':
                self.msg.next = 'FlyingState'
                self.state_machine_command.publish(self.msg)    
                
                
            rate.sleep()
          


if __name__ == '__main__':
	rospy.init_node('map', anonymous=True) #initializing the node
	kso = Key_State_Op()
	kso.menu()
	kso.run()
