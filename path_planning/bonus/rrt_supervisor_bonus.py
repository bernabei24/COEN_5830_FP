# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Display
from controller import Supervisor
from controller import Node
import struct
import time
import random

# Tunnable parameters for the RRT algorithm
AREA_SIZE_MAX_Width = 0.9
AREA_SIZE_MAX_Height = 0.9
AREA_SIZE_MIN_Width = 0.3
AREA_SIZE_MIN_Height = 0.3
goal = [0.6, 0.6, 0.0]
GOAL_COLLISION_THRESHOLD = 0.8


class Node:
    """
    A class to represent a node in the tree that holds the position, parent, action and duration of the robot at that node.
    """
    def __init__(self, position, parent, action, duration):
        self.position = position
        self.parent = parent
        self.action = action
        self.duration = duration
        
    def set_parent(self, parent):
        self.parent = parent    
    
    def get_parent(self):
        return self.parent
    
    def __str__(self):
        return str(self.position) + ' ' + str(self.action)
    
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
    if calculate_distance(position, goal) < GOAL_COLLISION_THRESHOLD:
        return True
    return False

 
    

def main():
     
    #Initialize the supervisor node, the graph and create a node class object 
    #to represent the initial state of the robot, which is also the only node in the tree
    supervisor = Supervisor()
 
    # Get initial position of the robot
    puck = supervisor.getFromDef("e-puck")
    initial_position = puck.getField('translation').getSFVec3f()
     
    root_node = Node(initial_position, None, [0.0, 0.0], 0.0)
    tree = [root_node]
    
    #Create variables for the e-puck robot, the emmiter and the receiver
    puck = supervisor.getFromDef("e-puck")
    emitter = supervisor.getDevice('emitter')
    receiver = supervisor.getDevice('receiver')
    
    #Set channels for the emitter reciever pair
    emitter.setChannel(10)
    receiver.enable(10)
    
    timestep = 32 #int(supervisor.getBasicTimeStep())
    
    #Use the emitter to request the puck's controller to perform monte carlo propagations
    receiver.enable(timestep)
   
    end_simulation = False
    
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
        set_position(supervisor, nearest_node.position, [0, 0, 1, 0])
    
        successful_propagations = []
        five_successful_propagations = False
        
        while not len(successful_propagations) == 5:
            supervisor.step(timestep)
            
            if receiver.getQueueLength()  == 0:
                continue
            
            data = receiver.getBytes()
            success, duration, left_motor_velocity, right_motor_velocity = struct.unpack('ffff', data)
            
            if success:
              position = puck.getPosition()

              node = Node(position, None, [left_motor_velocity, right_motor_velocity], duration)
              successful_propagations.append(node)
              
              five_successful_propagations = len(successful_propagations) 
              if five_successful_propagations == 5: 
                five_successful_propagations = True
                break

            
            else:
              message = struct.pack('f', float(0.0))
              emitter.send(message)
              break
              
        
             
        # position in successful_propagations closest to new_position
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
            
            # print goal reached in ascii art to the console
            print('Goal reached')
            
            # sleep for 10 seconds
            time.sleep(1)
            
            # set the position of the puck to the goal
            set_position(supervisor, initial_position, [0, 0, 1, 0])
            
            #print the final path
            final_path = []
            node = closest_node
            while node:
                final_path.append(node)
                node = node.get_parent()
            final_path.reverse()
            
            message = struct.pack('f', float(2.0))
            emitter.send(message)
            
            #Send the final path to the puck's controller
            # for node in final_path:
            for i  in range(len(final_path)):
                # create a message to send the final path to the puck's controller
                message = struct.pack('ffff', float(3.0), float(final_path[i].duration), final_path[i].action[0], final_path[i].action[1])
                print('Sending final velocities:', message)
                # byte length of the message
                print('Message length:', len(message))
                emitter.send(message)
                
            message = struct.pack('f', float(4.0))
            emitter.send(message)
            
            end_simulation = True
            
        
        if end_simulation:
            break           
        
            
        receiver.nextPacket()
             
    
            
if __name__ == "__main__":
    main()
