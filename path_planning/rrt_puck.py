"""rrt_puck controller."""
# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
import struct
import random
import time

 
TIME_STEP = 32 # milliseconds
 

def monte_carlo_propagate():
    # Initialize success to True
    #Set both the motors to velocity control mode
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    left.setPosition(float('inf'))
    right.setPosition(float('inf'))
    
    #Set the desired duration of random motion (in seconds)
    duration = random.randint(1, 10)

    #Assign a random velocity to the left motor
    #Assign a random velocity to the right motor
    left_speed = random.uniform(-5.28, 5.28)
    right_speed = random.uniform(left_speed-1, left_speed+1)
        
    #Enable the proximity sensors on the e-puck 
    proximity_sensors = []
    sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    timestep = TIME_STEP
        
    for name in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        proximity_sensors.append(sensor) 
     
    
    start_time = robot.getTime()
    left.setVelocity(left_speed)
    right.setVelocity(right_speed)
   
    while robot.step(timestep) != -1:
        
        
        sensor_values = [sensor.getValue() for sensor in proximity_sensors]
        for i,value in enumerate(sensor_values):
            #Perform collision checking at each iteration. Return False if collision occurs.
            if value > 100:
                # print collision time and sensor value
                # print('Collision time:', elapsed_time)
                # print('Sensor value:', value)
                return False, -1.0, [left_speed, right_speed]
            
        current_time = robot.getTime()
        elapsed_time = current_time - start_time
        if elapsed_time < duration:
            pass
        else:
            #If no collision occurs, stop the robot, return success and return the velocities.
            # print('No collision during random motion, returning velocities and SUCCESS') 
            left.setVelocity(0)
            right.setVelocity(0)
            current_time = robot.getTime()
            elapsed_time = current_time - start_time
            
            return True, elapsed_time, [left_speed, right_speed]
      
    
   
def replay_velocties(velocities):

    #Replay the final path on the robot
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    left.setPosition(float('inf'))
    right.setPosition(float('inf'))
    
    timestep = TIME_STEP 
    robot.step(timestep)
 
    while robot.step(timestep) != -1:
      for velocity in velocities:
        
        start_time = robot.getTime()
       
        print('Replaying velocities:', velocity)
        
        # Talked with Srikrishna about this part of the code
        # THIS IS NOT LOCALIZATION PROBLEM, AND THEREFORE
        # WE ARE NOT EXPECTING VELOCITY PLACKBACK TO BE PERFECT
        # AND WE DONT NEED TO PLAYBACK VELOCITIES, SO DONT!!!!!
        #
        #
        # left.setVelocity(velocity[2] )
        # right.setVelocity(velocity[3] )
        
        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            elapsed_time = current_time - start_time
            # print('Elapsed time:', elapsed_time)
            if elapsed_time <= (velocity[1]):
               pass
            else:
            #    left.setVelocity(0)
            #    right.setVelocity(0)
               break
           
        start_time = robot.getTime()  
        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            elapsed_time = current_time - start_time
            if elapsed_time < 1:
              right.setVelocity(0)
              left.setVelocity(0)
            else:
              break
          
       
      break 
  
    # stop the robot
    left.setVelocity(0)
    right.setVelocity(0)
            
        


robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize the Webots Supervisor
supervisor = robot.getSupervisor()


#Create variables for the e-puck robot, the emmiter and the receiver
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')

#Set channels for the emitter reciever pair
emitter.setChannel(10)
receiver.enable(10)

GOAL_REACHED = False
REPLAY_COMPLETE = False

final_velocities = []

# Run the simulation step by step in a while loop:
while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        # print('Received message:', message)
        receiver.nextPacket()
        
        # Unpack the message first 4 bytes
        messageStart = struct.unpack('f', message[:4])
        
        #If data asks for monte carlo propagation:
        if messageStart == (1.0,) and not GOAL_REACHED:
            #perform monte carlo propagation from the robot's current state until 5 successful propagations are obtained.
            success, duration, action = monte_carlo_propagate()
            print(success, action, type(action))
            data_bytes = struct.pack('ffff', float(success), float(duration), action[0], action[1])
            # Use the emmiter to send the following after each propagation attempt: 
            # success of propagation (boolean), left motor velocity and right motor velocity.
            emitter.send(data_bytes)
            
            
##########################################################################################################
# If data indicates that the goal has been reached, 
# then write code to find the final path and replay the final 
# set of actions on the robot all the way from its initial state to the goal state
##########################################################################################################            
        elif messageStart == (2.0,) and not GOAL_REACHED:
            GOAL_REACHED = True
            print('Goal reached')
            
        elif messageStart == (3.0,):
            print('Receiving final path')
            
            #left_motor.setVelocity(0.0)
            #right_motor.setVelocity(0.0)
            
            # Receive the final path from the supervisor
            velocities = struct.unpack('ffff', message)
            print('Final velocities:', velocities)
            
            # add last 2 velocities to final velocities
            final_velocities.append(velocities)

            #left_motor.setVelocity(velocities[0])
            #right_motor.setVelocity(velocities[1])
            # Replay the final path on the robot
            
        elif messageStart == (4.0,):
            replay_velocties(final_velocities)
            print('Replay complete')
            emitter.send(struct.pack('f', float(5.0)))
            break
             
             
##########################################################################################################
# End of code to replay the final set of actions on the robot
########################################################################################################## 
       
            