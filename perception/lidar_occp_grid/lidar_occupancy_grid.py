import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import csv


"""
  COEN 5830 - Intro to Robotics
  Final Project: Lidar Occupancy Grid Mapping
  Student(s): Michael Bernabei, Ian Mcconachie                                                                       
  Fall 2024
  Instructor: Dr. Leopold Beuken
                                                                                                                                                                
  Program description:  The is program takes lidar data generates a 2d occupancy grid map. 

  Author(s): Michael Bernabei, Ian Mcconachie
  Date: 11/10/2024
  
  precondition: The data.csv file must be in the same directory as this script.  
  
"""

# Constants
GRID_BUFFER = 5
GRID_RESOLUTION = .05
LIDAR_COLLISON_COLOR = 100

"""
  The GridMap class is used to create and update an occupancy grid map based on lidar data.
"""
class GridMap:
    
    
    def __init__(self, resolution = 1, default_background=128):
        self.resolution = resolution
        self.default_background = default_background
        self.grid_x_max = 0
        self.grid_x_min = 0
        self.grid_y_max = 0
        self.grid_y_min = 0
        self.robo_positions = []
    
    def create_grid(self, width, height):
        self.width = width / self.resolution
        self.height = height / self.resolution
        self.width = int(self.width)
        self.height = int(self.height)
        self.grid = [[self.default_background for _ in range(self.width)] for _ in range(self.height)]
        
    def create_grid_from_data(self, robo_pos_x, robo_pos_y, robo_heading, robo_lidar):
        """
        Creates a grid based on the lidar data and robot position data.  That is,
        the grid will be large enough to encompass the entire area that the robot
        has traveled. And large enough to encompass the entire area that the lidar
        has been able to detect.
        
        Parameters:
          robo_pos_x (list): list of x positions of the robot
          robo_pos_y (list): list of y positions of the robot
          robo_heading (list): list of headings of the robot
          robo_lidar (list): list of lidar data
        
        """
        self.robo_pos_x = robo_pos_x
        self.robo_pos_y = robo_pos_y
        self.robo_heading = robo_heading
        self.robo_lidar = robo_lidar

        lidar_distances_x = []
        lidar_distances_y = []
        
        # It is almost a gaurantee that the min and max values in 
        # the real space will be from lidar readings.  Hence
        # our bounds for the grid will be based off min and max
        # lidar readings
        for i in range(len(self.robo_pos_x)):
            for j in range(720):
                if self.robo_lidar[i][j] == float('inf'):
                    continue

                x_ = self.robo_pos_x[i] + self.robo_lidar[i][j] * np.cos(self.robo_heading[i] + np.pi + np.deg2rad(j*(360/720)))
                y_ = self.robo_pos_y[i] + self.robo_lidar[i][j] * np.sin(self.robo_heading[i] + np.pi + np.deg2rad(j*(360/720)))
                lidar_distances_x.append(x_)
                lidar_distances_y.append(y_)
                
        
        self.grid_x_max = abs(max(lidar_distances_x) + abs(min(lidar_distances_x))) + GRID_BUFFER
        self.grid_x_min = min(lidar_distances_x) - GRID_BUFFER
        self.grid_y_max = abs(max(lidar_distances_y) + abs(min(lidar_distances_y))) + GRID_BUFFER
        self.grid_y_min = min(lidar_distances_y) - GRID_BUFFER
        
        self.width = self.grid_x_max / self.resolution
        self.height = self.grid_y_max / self.resolution
        self.width = int(self.width) + 1
        self.height = int(self.height) + 2
        self.grid = [[self.default_background for _ in range(self.width)] for _ in range(self.height)]
        
        
    def bresenham(self, start, end):
        """
        The heart of the occupancy grid mapping algorithm is the Bresenham's line drawing algorithm. 
        Below is an implementation of the algorithm.
        
        The algorithim psuedo code is housed on wikipedia: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        
        dx = x2 - x1
        dy = y2 - y1
        
        # Determine if the line needs to be flipped
        line_flip_needed = abs(dy) > abs(dx)
        if line_flip_needed:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        line_swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            line_swapped = True
        
        dx = x2 - x1
        dy = y2 - y1
        error = int(dx / 2.0)
        y_step = 1 if y1 < y2 else -1
      
  
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if line_flip_needed else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
             y += y_step
             error += dx
        if line_swapped:
            points.reverse()
        points = np.array(points)
      
        return points
 

    def set_value(self, x, y, value):
        """
        Set the value of a cell in the grid
    
        Parameters:
        x (int): x coordinate
        y (int): y coordinate
        value (int): grayscale value to set the cell to
        """
        if (x,y) in self.robo_positions:
            return
        
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = value
        else:
            print('Out of bounds' + str(x) + ' ' + str(y))


    def set_occupied(self, coordinates, lidar_val=-1):
        """
        Set the value of a cell in the grid from 0 to 255 (occupied)
        """
        for x, y in coordinates:
            self.set_value(x, y, 255)
        
        if lidar_val != -1:
            self.set_value(coordinates[-1][0], coordinates[-1][1], LIDAR_COLLISON_COLOR)
    
    def get_grid(self):
        return np.array(self.grid)
    
    
    def draw_grid(self):
        """
        Print the grid to the console
        """
        for i in range(self.width):
            for j in range(self.height):
                print(self.grid[i][j], end=' ')
            
            
    def mark_robot_path(self, x, y):
        """
        Draw the robot path on the grid map
        """
        _x, _y = self.calculate_grid_coords( x, y)
        self.set_value(_x, _y, 0)
        self.robo_positions.append((_x, _y))

    def calculate_grid_coords(self, x, y):
        """
        Calculate the grid coordinates for a given x, y position
        """
        grid_x = int(np.round((x - self.grid_x_min) / self.resolution))
        grid_y = int(np.round((y - self.grid_y_min) / self.resolution))
        return grid_x, grid_y
    
    def lidar_measurement_update(self,lidar_data, pos_x, pos_y, heading):
        """
        Update the occupancy grid map based on lidar data
        """
        for l in range(720):
         if lidar_data[l] == float('inf'):
            continue
         x_ = pos_x + lidar_data[l] * np.cos(heading + np.pi + np.deg2rad(l*(360/720)))
         y_ = pos_y + lidar_data[l] * np.sin(heading + np.pi + np.deg2rad(l*(360/720)))
         grid_lidar_x, grid_lidar_y = self.calculate_grid_coords(x_, y_)
         grid_pos_x, grid_pos_y = self.calculate_grid_coords(pos_x, pos_y)
         points = self.bresenham((grid_pos_x, grid_pos_y), (grid_lidar_x, grid_lidar_y))
         if lidar_data[l] != float('inf'):
           self.set_occupied(points,lidar_data[l])
         else:
           self.set_occupied(points)
           
    def calculate_x_ticks(self):
        """
        Rescales the grid x axis to match real world coordinates.
        """
        x_ticks = []
        x_labels = []
        
        steps = 10
        
        if self.grid_y_min < 0:
            grid_x_zero = int(np.round((0 - self.grid_x_min) / self.resolution))
            x_ticks.append(grid_x_zero)
            x_labels.append(0)
            
            for i in range(grid_x_zero, self.width, int((self.width/steps) )):
                x_ticks.append(round(i))
                x_labels.append(round(self.grid_x_min + i * self.resolution))
                
            for i in range(grid_x_zero, 0, -int((self.width/steps) )):
                x_ticks.append(round(i))
                x_labels.append(round(self.grid_x_min + i * self.resolution))
        else:
            for i in range(0, self.width, int((self.width*self.resolution) )):
                x_ticks.append(round(i))
                x_labels.append(round(self.grid_x_min + i * self.resolution))
 
        
        return x_ticks, x_labels
  
    def calculate_y_ticks(self):
        """
        Rescales the grid y axis to match real world coordinates.
        """
        y_ticks = []
        y_labels = []
        
        steps = 10 
  
        if self.grid_y_min < 0:
            grid_y_zero = int(np.round((0 - self.grid_y_min) / self.resolution))
            y_ticks.append(grid_y_zero)
            y_labels.append(0)
            
            for i in range(grid_y_zero, self.height, int((self.height/steps ) )):
                if round(i) in y_ticks:
                    continue
                y_ticks.append(round(i))
                y_labels.append(round(self.grid_y_min + i * self.resolution))
                
            for i in range(grid_y_zero, 0, -int((self.height/steps) )):
                y_ticks.append(round(i))
                y_labels.append(round(self.grid_y_min + i * self.resolution))
        else:
            for i in range(0, self.height, int((self.height/steps) )):
                y_ticks.append(round(i))
                y_labels.append(round(self.grid_y_min + i * self.resolution))
                
        return y_ticks, y_labels
 
        
         

############################################################################################################
# Data loading Helper functions
############################################################################################################

def read_csv_file(file_path):
    data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            data.append(row)
            
    x = [row[0] for row in data]
    y = [row[1] for row in data]

    heading = [row[2] for row in data]

    lidar_data = [row[3:] for row in data]

    #convert string to float
    x = [float(i) for i in x]
    y = [float(i) for i in y]
    heading = [float(i) for i in heading]
    lidar_data = [[float(i) for i in row] for row in lidar_data]
    
    return x,y,heading,lidar_data

############################################################################################################
# End Data loading Helper functions
############################################################################################################




############################################################################################################
# Animation Helper functions
############################################################################################################

# Create a figure and axes
fig, ax = plt.subplots()
 
res = GRID_RESOLUTION
pos_x, pos_y, heading, lidar_data = read_csv_file('data.csv')
grid_map = GridMap(resolution=res)
grid_map.create_grid_from_data(pos_x, pos_y, heading, lidar_data)
grid_map.mark_robot_path(pos_x[0], pos_y[0])

# Create an image plot
im = ax.imshow(grid_map.get_grid(), cmap='gray', vmin=0, vmax=255)
ax.invert_yaxis()

# ax.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F'])
x_ticks, x_labels = grid_map.calculate_x_ticks()
ax.set_xticks(x_ticks)
ax.set_xticklabels(x_labels)
ax.set_xlabel('X [meters]')

y_ticks, y_labels = grid_map.calculate_y_ticks()
ax.set_yticks(y_ticks)
ax.set_yticklabels(y_labels)
ax.set_ylabel('Y [meters]')
ax.set_title('Occupancy Grid Map with resolution set to ' + str(res) + ' meters')


# Update function for the animation - this is called for each frame of the animation
def update(frame, x, y, heading, lidar_data):
    grid_map.mark_robot_path(x[frame], y[frame])
    grid_map.lidar_measurement_update(lidar_data[frame], x[frame], y[frame], heading[frame])
 
    im.set_data(grid_map.get_grid())
    return [im]


# Create the animation object - this will call the update function for each frame
ani = animation.FuncAnimation(fig, update, frames=199, interval=1, fargs=(pos_x, pos_y, heading, lidar_data),blit=True, repeat=False)

# Save the animation as a GIF
#animation_file = 'lidar_mapping_with_'+ 'res_set_to_' + str(res)+'.gif'
#ani.save(animation_file, writer='pillow')

# Show the animation
plt.show()

############################################################################################################
# End Animation Helper functions
############################################################################################################

    