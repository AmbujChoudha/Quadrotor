class Key_State_Op():
	def menu(): 
		print "Press"
		print "A: to set mode to ARM the drone"
	    print "S: to set mode to STABILIZE"
	    print "D: to set mode to DISARM the drone"
	    print "T: to set mode to TAKEOFF"
	    print "L: to set mode to LAND"
	    print "LO: print GPS coordinates"

	def __init__():
		self.state_machine_command=rospy.Publisher('state_machine/command',std_msgs::String)

	def run():
	    x='1'
	    while ((not rospy.is_shutdown())and (x in ['A','S','D','T','L','LO'])):
	        self.menu()
	        x = raw_input("Enter your input: ");
	        if (x=='A'):
	            self.state_machine_command.publish("Arming");
	        
	        else: 
	            print "Exit"


def __main__():
	rospy.init_node(...)
	kso=Key_State_Op()
	kso.run()