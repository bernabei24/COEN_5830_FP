import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# TODO - figure out if this is needed,  we integrate the gyro estimates in the filter
#        starting from 0, hence do we need to transform to Euler rates?
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

##########################################################################################
# End of data loading
##########################################################################################


##########################################################################################
# Complimentary filter initialization
##########################################################################################

dt = time[1] - time[0]
samples = len(time) 

# Hold yaw, pitch, and roll values  
euler_angles = np.zeros((samples, 3))

# We prefer the gyroscope data over the accelerometer 
# data by sleceting a high value for ALPHA
ALPHA = 0.98

gyro_deg_to_rad = np.pi / 180   




##########################################################################################
# End of complimentary filter initialization
##########################################################################################

"""
 Complimentary filter algorithm (A Sensor Fusion Algorithm)  
 
   We use the complimentary filter algorithim to combine/fuse the accelerometer and gyroscope data.
   And therefore get a more accurate estimation of the orientation of the IMU.  In our
   code we favor the gyroscope data over the accelerometer data by using a high value for ALPHA.
   

"""
gyro_x_rad = gyro_x[0] * gyro_deg_to_rad
gyro_y_rad = gyro_y[0] * gyro_deg_to_rad
gyro_z_rad = gyro_z[0] * gyro_deg_to_rad

euler_phi = 0
euler_theta = 0

for i in range(1, samples):
    
    # convert the gyroscope data to radians
    gyro_x_rad = gyro_x[i] * gyro_deg_to_rad
    gyro_y_rad = gyro_y[i] * gyro_deg_to_rad
    gyro_z_rad = gyro_z[i] * gyro_deg_to_rad
    
  
    # previous yaw, pitch, and roll values
    roll, pitch, yaw = euler_angles[i-1]
    
    #Integrate the gyroscope data using Euler's method
    roll += gyro_x_rad * dt
    pitch += gyro_y_rad * dt
    yaw += gyro_z_rad * dt
    
    
    # Accelerometer data processing
    acc_angle_x = np.arctan2(accel_y[i], accel_z[i])  
    acc_angle_y = np.arctan2(-accel_x[i], np.sqrt(accel_y[i] ** 2 + accel_z[i] ** 2))  
    
    # Complimentary filter
    roll = ALPHA * roll + (1 - ALPHA) * acc_angle_x
    pitch = ALPHA * pitch + (1 - ALPHA) * acc_angle_y
    
    euler_angles[i] = [roll, pitch, yaw]
    
# Plot the data
euler_angles_deg = np.degrees(euler_angles)


#print last values in euler_angles_deg
print('Roll, Pitch, Yaw: ' , euler_angles_deg[-1], ' (degrees)')


plt.figure(figsize=(15,5))
plt.subplot(3,1,1)
plt.plot(time, euler_angles_deg[:,0], label='Roll')
plt.title('Roll')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.grid()
 
plt.subplot(3,1,2)
plt.plot(time, euler_angles_deg[:,1], label='Pitch')
plt.title('Pitch')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.grid()
 
plt.subplot(3,1,3)
plt.plot(time, euler_angles_deg[:,2], label='Yaw')
plt.title('Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.grid()
 
plt.tight_layout()
plt.show()




 
    
    
    