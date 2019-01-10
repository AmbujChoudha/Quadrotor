# Quadrotor
The description of this repository is following:<br /> 
**multi_quadrotors**: bash scripts launch multiple quadrotrs in in Gazebo platform.<br /> 
**single_quadrotors**: bash scripts launch single quadrotors in the Gazebo platform.<br /> 
**python_scripts**: It has two files (1) state_machine.py (2) states.py <br /> 
(1) **state_machine.py**: The idea of the state machine is to control all states, add new states and update current states. The code works following way: <br /> 
(2) **states.py**: has the descriptions of the states. <br /> 
[note: -> program propagation] 

(i) Program starts from sm = StateMachine(): is creating StateMachine instance ->  def __init__(self): all the constructor node is being initialized here. <br /> 

(ii) -> def __init__(self): first it is creating instance of  the publisher class cmd_vel_topic which will publish a string ROS msg. The program also creates an instance of subscriber class change_state_topic which is subscribing to ROS topic "change_state".<br /> 

(iii) Inside the for loop, the program is iterating through states.py. For loop is first verifying inspect.getmembers(states,inspect.isclass): is there any class. If there the class name which is a string in the dictionary call the _add_state function with class name as string argument for the function. <br /> 

(iv) -> def _add_state(self, new_state):  in this function the program is adding the new state/class. First the function is checking whether new_state in the list or not. If not in the list, first initialize the instance. Inside the try block, the function is calling the .next function inside a class/state for example instance.next() in GroundedState() after verifying the whether the instance.next is callable or not. Inside this if block the function also publishing ROS msg in the topic "cmd_vel_topic". <br /> 

(v) -> the program then goes back to for loop do the same as steps 1 and steps 2. Once it’s done with adding all the states/class in the list, the program run this self.change_state('GroundedState', ''). <br /> 

(vi) -> with 'GroundedState' as argument, initially the change state is updating the current state as 'GroundedState'.<br /> 

(vii) -> sm.run() will run the current state self.states[self.current_state].run() which is running the base class run function: def run(self):   self.cmd_vel.publish(self.msg) which is publishing the default parameter of the Twistamped message (0,0,0). <br /> 

(viii) -> after this initial initialization process, if subscriber node subscribes to a topic with new state, _change_state_wrapper will be called with the ROS message from the topic. _change_state_wrapper  calls the change_state function with old state and new state as a parameter. Change state adds new state if it does not exist, then update the current state. Sm.run will run with new current state class.  <br /> 

