# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Node
import struct
import time
import random

AREA_SIZE_MAX_Width = 0.9
AREA_SIZE_MAX_Height = 0.9
AREA_SIZE_MIN_Width = 0.3
AREA_SIZE_MIN_Height = 0.3

goal = [0.6, 0.6, 0.0]

#Create a class to represent a Node, which consists of position, parent Node and action(velocity of the motors)
class Node:
    def __init__(self, position, parent, action):
        self.position = position
        self.parent = parent
        self.action = action
        
    def set_parent(self, parent):
        self.parent = parent    
    
    def get_parent(self):
        return self.parent
    
def return_node_closest_to_position(position, tree):
    #Iterate through the tree to find the node closest to the position
    min_distance = 1000
    closest_node = None
    for node in tree:
        distance = calculate_distance(node.position, position)
        if distance < min_distance:
            min_distance = distance
            closest_node = node
    return closest_node
     
 
def calculate_distance(position1, position2):
    return ((position1[0] - position2[0])**2 + (position1[1] - position2[1])**2)**0.5
    

def randomly_sample_state(supervisor):
    print('Randomly sampling state')
    #Using 50% goal biasing, sample a state within the arena and return the state
    new_position = [random.uniform(AREA_SIZE_MIN_Width, AREA_SIZE_MAX_Width), random.uniform(AREA_SIZE_MIN_Height, AREA_SIZE_MAX_Height), 0.0]    
    
    return new_position
            
def set_position(supervisor, position, orientation):
    puck = supervisor.getFromDef("e-puck")
    puck.getField('translation').setSFVec3f(position)
    puck.getField('rotation').setSFRotation(orientation)    
    
def check_if_goal_reached(position):
    #Check if the position is close enough to the goal
    if calculate_distance(position, goal) < 0.15:
        return True
    return False


def main():
     
    #Initialize the supervisor node, the graph and create a node class object to represent the initial state of the robot, which is also the only node in the tree
    supervisor = Supervisor()
    
    # Get initial position of the robot
    puck = supervisor.getFromDef("e-puck")
    initial_position = puck.getField('translation').getSFVec3f()
     
    
    root_node = Node(initial_position, None, [0.0, 0.0])
    tree = [root_node]
    
    
    #Create variables for the e-puck robot, the emmiter and the receiver
    puck = supervisor.getFromDef("e-puck")
    emitter = supervisor.getDevice('emitter')
    receiver = supervisor.getDevice('receiver')
    
    #Set channels for the emitter reciever pair
    emitter.setChannel(10)
    receiver.enable(10)
    
    timestep = int(supervisor.getBasicTimeStep())
    
    #Use the emitter to request the puck's controller to perform monte carlo propagations
    receiver.enable(timestep)
    
    # Run the simulation step by step in a while loop:
    while supervisor.step(timestep) != -1:
        
        #Randomly sample a state
        new_position = randomly_sample_state(supervisor)
        
        # puck.getField
        message = struct.pack('f', float(1.0))
        # print('Sending message:', message)
        emitter.send(message)
         
        #Find the nearest node in the tree to the randomly sampled state
        nearest_node = return_node_closest_to_position(new_position, tree)
        
        # set the position of the puck to the nearest node
        set_position(supervisor, nearest_node.position, [0, 1, 0, 0])
    
        
        successful_propagations = []
        five_successful_propagations = False
        
        while not len(successful_propagations) == 5:
            supervisor.step(timestep)
            
            if receiver.getQueueLength()  == 0:
                continue
            
            data = receiver.getBytes()
            
            
            success, left_motor_velocity, right_motor_velocity = struct.unpack('fff', data)
            if success:
              print('Success in propagation')
              # save robot position
              position = puck.getPosition()
 
            #   print('Position:', position)
            #   print('left motor velocity:', left_motor_velocity)    
            #   print('right motor velocity:', right_motor_velocity)
     
              node = Node(position, None, [left_motor_velocity, right_motor_velocity])
              successful_propagations.append(node)
              
              five_successful_propagations = len(successful_propagations) 
              if five_successful_propagations == 5:
                    print('Five successful propagations')
                    five_successful_propagations = True
                    
              
              print('Propagation length:', len(successful_propagations))
            
            else:
              print('Collision detected by the supervisor')
              message = struct.pack('f', float(0.0))
              print('Sending message:', message)
              emitter.send(message)
              break
              
        
             
        # position in successful_propagations closed to new_position
        if five_successful_propagations:
          closest_node = return_node_closest_to_position(new_position, successful_propagations)
          print('Closest node:', closest_node.position)
          print('New position:', new_position)
        
          #Add the closest node to the tree
          closest_node.set_parent(nearest_node)
          tree.append(closest_node)
          print('Tree length:', len(tree))
          
          #Check if the goal has been reached
          if check_if_goal_reached(closest_node.position):
            print('Goal reached!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            #sleep for 1 hour
            #time.sleep(3600)
            break    
            
        receiver.nextPacket()
        time.sleep(.015)    
    
    
    
    	
        #While 5 succcesfull propagations are yet to be executed:
            #Use the reciever to obtain the success and the velocity values
            #If success if false, collision has occured. Move on.
            #Else, obtain the final position of the robot and store it.
            #Teleport the puck back to the nearest node
        #Compare the final positions with the randomly sampled state and pick the nearest one.
        #Add it to the tree
        #Is the new state close enough to the goal:
        	#GOAL REACHED! 
        	#Write code to gather the final set of actions and use emitter to transmit data for replaying
            
if __name__ == "__main__":
    main()
