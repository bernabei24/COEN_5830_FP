import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
  COEN 5830 - Intro to Robotics
  Final Project: 1.3 IMU filtering 
  Student(s): Michael Bernabei, Ian Mcconachie                                                                       
  Fall 2024
  Instructor: Dr. Leopold Beuken
                                                                                                                                                                
  Program description: uses the complimentary filter to get a more accurate estimation for the euler angles

  Author(s): Michael Bernabei, Ian Mcconachie
  Date: 12/13/2024
  
  precondition: The imu_data.csv file must be in the same directory as this script.  
  
"""
# not used in this case since it is computationally expensive for realtime estimation and only adds marginal benefit 
def body_rates_to_euler_rates(phi, theta, p,q,r):
    
    # Calculate the transformation matrix
    transformation_matrix = np.array([[1, 0, np.sin(theta)],
                  [0, np.cos(phi), -np.cos(theta)*np.sin(phi)],
                  [0, np.sin(phi), np.cos(theta)*np.cos(phi)]])
    
    transformation_matrix = np.linalg.inv(transformation_matrix)
    
    body_rates = np.array([p, q, r])
    
    euler_rates = transformation_matrix @ body_rates
    
    return euler_rates

##########################################################################################
# Load the data
##########################################################################################
data = pd.read_csv('imu_data.csv')

time = data['Time (s)'].values
accel_x = data['Accel X'].values
accel_y = data['Accel Y'].values
accel_z = data['Accel Z'].values
gyro_x = data['Gyro X '].values
gyro_y = data['Gyro Y '].values
gyro_z = data['Gyro Z'].values

"""
 Complimentary filter algorithm   
 
   We use the complimentary filter algorithm to combine/fuse the accelerometer and gyroscope data.
   And therefore get a more accurate estimation of the orientation of the IMU.  In our
   code we favor the gyroscope data over the accelerometer data by using a high value for alpha.
   
"""
##########################################################################################
# Complimentary filter 
##########################################################################################

def Complimentary_filter (time, accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z, alpha):
    dt = time[1] - time[0]
    samples = len(time) 

    # Hold yaw, pitch, and roll values  
    euler_angles = np.zeros((samples, 3))

    # We prefer the gyroscope data over the accelerometer 
    # data by selecting a high value for ALPHA
    ALPHA = alpha

    for i in range(1, samples):
        
        # previous yaw, pitch, and roll values
        roll, pitch, yaw = euler_angles[i-1]
        
        #Integrate the gyroscope data using Euler's method
        roll += (gyro_x[i]+gyro_x[i-1])/2 * dt
        pitch += (gyro_y[i]+gyro_y[i-1])/2 * dt
        yaw += (gyro_z[i]+gyro_z[i-1])/2 * dt
        
        # Accelerometer data processing
        acc_roll = np.arctan2(accel_y[i], np.sqrt(accel_y[i] ** 2 + accel_z[i] ** 2))  
        acc_pitch = np.arctan2(np.sqrt(accel_x[i] ** 2 + accel_y[i] ** 2), accel_z[i])  

        roll = ALPHA * roll + (1 - ALPHA) * acc_roll
        pitch = ALPHA * pitch + (1 - ALPHA) * acc_pitch
        
        euler_angles[i] = [roll, pitch, yaw]
        
    return euler_angles
    
euler_angles_a_1 = Complimentary_filter (time, accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z, 1)
euler_angles_a_0 = Complimentary_filter (time, accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z, 0)
euler_angles_a_95 = Complimentary_filter (time, accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z, 0.95)

##########################################################################################
# Plot
##########################################################################################
RadToDeg = 180/np.pi

plt.figure(figsize=(10,6))
plt.subplot(3,1,1)
plt.title('Complimentary filter estimation of euler angles, alpha = 0.95')
plt.plot(time, euler_angles_a_1[:,0] * RadToDeg, "g:" ,label='Gyroscope estimation')
plt.plot(time, euler_angles_a_0[:,0] * RadToDeg, "r--" , label='Accelerometer estimation')
plt.plot(time, euler_angles_a_95[:,0] * RadToDeg, "b-" , label='Complimentary filter estimation')
plt.xlim([0, time[-1]])
plt.ylabel('Roll angle (°)')
plt.grid()
plt.legend()
 
plt.subplot(3,1,2)
plt.plot(time, euler_angles_a_1[:,1] * RadToDeg, "g:" ,label='Gyroscope estimation')
plt.plot(time, euler_angles_a_0[:,1] * RadToDeg, "r--" , label='Accelerometer estimation')
plt.plot(time, euler_angles_a_95[:,1] * RadToDeg, "b-" , label='Complimentary filter estimation')
plt.xlim([0, time[-1]])
plt.ylabel('Pitch angle (°)')
plt.grid()
 
plt.subplot(3,1,3)
plt.plot(time, euler_angles_a_1[:,2] * RadToDeg, "g:" ,label='Gyroscope estimation')
plt.plot(time, euler_angles_a_0[:,2] * RadToDeg, "r--" , label='Accelerometer estimation')
plt.plot(time, euler_angles_a_95[:,2] * RadToDeg, "b-" , label='Complimentary filter estimation')
plt.xlabel('Time (s)')
plt.xlim([0, time[-1]])
plt.ylabel('Yaw angle (°)')
plt.grid()
 
plt.tight_layout()
plt.show()
