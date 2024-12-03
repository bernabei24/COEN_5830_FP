"""rrt_puck controller."""


# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
import struct
import random
import time

def monte_carlo_propagate():
    # Initialize success to True
    #Set both the motors to velocity control mode
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    left.setPosition(float('inf'))
    right.setPosition(float('inf'))
    
   
    #Set the desired duration of random motion (in seconds)
    duration = random.randint(1, 3)
    #duration = 30
     
    
     
    #Assign a random velocity to the left motor
    #Assign a random velocity to the right motor
    left_speed = random.uniform(-5.28, 5.28)
    right_speed = random.uniform(left_speed-1, left_speed+1)
    left.setVelocity(left_speed)
    right.setVelocity(right_speed)
    
    #Enable the proximity sensors on the e-puck 
    proximity_sensors = []
    sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    
    timestep = 32
    
    
    for name in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        proximity_sensors.append(sensor) 
     
    
    start_time = robot.getTime()
   
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        elapsed_time = current_time - start_time
        
        
        
        
        sensor_values = [sensor.getValue() for sensor in proximity_sensors]
        for i,value in enumerate(sensor_values):
            #Perform collision checking at each iteration. Return False if collision occurs.
            if value > 300:
                # print collision time and sensor value
                # print('Collision time:', elapsed_time)
                # print('Sensor value:', value)
                return False, [left_speed, right_speed]
                 
        
        # Generate random velocities for the left and right motors every 1 second
        if elapsed_time % .5 == 0:
          left_speed = random.uniform(-5.28, 5.28)
          right_speed = random.uniform(left_speed-1, left_speed+1)
          left.setVelocity(left_speed)
          right.setVelocity(right_speed)
   
        
        if elapsed_time < duration:
            pass
        else:
            #If no collision occurs, stop the robot, return success and return the velocities.
            print('No collision during random motion, returning velocities and SUCCESS') 
            left.setVelocity(0)
            right.setVelocity(0)
            return True, [left_speed, right_speed]
        
        time.sleep(0.015)
    
   
   
   
    
    
    
   
    
    


#main
#Initialize the robot, obtain variables for the emitter and receiver on the robot.
#Set emitter and receiver channels
#In a while loop:
	#Check if receiver has received data
	#If data asks for monte carlo propagation:
		#perform monte carlo propagation from the robot's current state until 5 successful propagations are obtained.
		#Use the emmiter to send the following after each propagation attempt: success of propagation (boolean), left motor velocity and right motor velocity.
        #If data indicates that the goal has been reached, then write code to find the final path and replay the final set of actions on the robot all the way from its initial state to the goal state


# move webots e-puck robot
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

# Run the simulation step by step in a while loop:
while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        # print('Received message:', message)
        receiver.nextPacket()
        message = struct.unpack('f', message)
        # print('Unpacked message:', message)
        if message == (1.0,):
            # print('Message is,', message)
            success, action = monte_carlo_propagate()
            print(success, action, type(action))
            data_bytes = struct.pack('fff', float(success), action[0], action[1])
            emitter.send(data_bytes)