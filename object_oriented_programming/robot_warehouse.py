import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
 

robot_types = ['QUADCOPTER', 'DIFFERENTIAL_DRIVE', 'HUMANOID']

legal_moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

class Djikstra:
    def __init__(self, width = 10, height = 10):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))
        self.grid.fill(np.inf)  
        
    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.grid[y, x] = 0
    
    def set_start(self, x, y):
        self.start_x = x
        self.start_y = y
        self.grid[y, x] = 0
    
    def update_grid(self, x, y, value):
        self.grid[y, x] = value
    
    def get_grid(self):
        return self.grid
    
    def get_neighbors(self, x, y):
        neighbors = []
        for move in legal_moves:
            new_x = x + move[0]
            new_y = y + move[1]
            if new_x >= 0 and new_x < self.width and new_y >= 0 and new_y < self.height:
                neighbors.append((new_x, new_y))
        return neighbors
    
    def get_shortest_path(self, warehouse):
        shortest_path = []
        
        x = self.start_x
        y = self.start_y
        
        unvisited = []
        visited = [] 
        min_distance = dict()
        
        # mark obstacles as visited
        obstacles = warehouse.get_obstacles()
        
        # mark obstacles as visited
        for obstacle in obstacles:
            visited.append(obstacle)
            min_distance[obstacle] = np.inf
        
        
        for x,y in np.ndindex(self.grid.shape):
            unvisited.append((x, y))
            min_distance[(x, y)] = np.inf
            
        min_distance[(self.start_x, self.start_y)] = 0
        
        while unvisited:
            min_node = min(unvisited, key=lambda x: min_distance[x])
            unvisited.remove(min_node)
            visited.append(min_node)
            
            if min_node[0] == self.goal_x and min_node[1] == self.goal_y:
                break
            
            neighbors = self.get_neighbors(min_node[0], min_node[1])
            
            # check if neighbor is an obstacle
            for obstacle in obstacles:
                if obstacle in neighbors:
                    neighbors.remove([4,3])
            
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                
                if neighbor in obstacles:
                    print(obstacles)
                    continue
                
                distance = min_distance[min_node] + 1
                
                if distance < min_distance[neighbor]:
                    min_distance[neighbor] = distance
                    
        # reconstruct path
        x = self.goal_x
        y = self.goal_y
        
        shortest_path.append((x, y))
        
        while (x, y) != (self.start_x, self.start_y):
            neighbors = self.get_neighbors(x, y)
            for neighbor in neighbors:
                if min_distance[neighbor] == min_distance[(x, y)] - 1:
                    shortest_path.append(neighbor)
                    x = neighbor[0]
                    y = neighbor[1]
                    break
                
        return shortest_path
    
        
class Robot:
    def __init__(self, x, y, robot_type, warehouse):
        self.x = x
        self.y = y
        self.type = robot_type
        self.warehouse = warehouse
        self.move_request_x = 0
        self.move_request_y = 0
        
        if robot_type == 'QUADCOPTER':
            self.color = 'red'
        elif robot_type == 'DIFFERENTIAL_DRIVE':
            self.color = 'green'
        elif robot_type == 'HUMANOID':
            self.color = 'blue'
        
    
    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
    
    def set_position(self, x, y):
        self.x = x
        self.y = y  
    
    def get_shortest_path(self):
        djikstra = Djikstra()
        djikstra.set_goal(self.goal_x, self.goal_y)
        djikstra.set_start(self.x, self.y)
        shortest_path = djikstra.get_shortest_path(self.warehouse)
        return shortest_path
    
    
    
    def move_towards_goal(self):
        shortest_path = self.get_shortest_path()
        
        if len(shortest_path) == 0:
            return
        elif len(shortest_path) > 1:
            move_x_request = shortest_path[-2][0]
            move_y_request = shortest_path[-2][1]
        else:
            move_x_request = shortest_path[-1][0]
            move_y_request = shortest_path[-1][1]
            
        
        self.move_request_x = move_x_request
        self.move_request_y = move_y_request
        
    def get_move_request(self):
        return self.move_request_x, self.move_request_y
    
    
         
        
    
        

class Warehouse:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.warehouse = np.zeros((width, height))
        
        # set warehouse background color to white
        self.warehouse.fill(255)
        
        self.robots = []
    
    def add_robot(self, robot):
        self.robots.append(robot)
        
    def remove_robot(self, robot):
        # check if robot is in warehouse
        if robot in self.robots:
          self.robots.remove(robot)
    
    def move_robot(self, robot, x, y):
        # check if the new position is within the warehouse
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        
        # check if the new position is occupied by another robot
        if self.is_position_occupied(x, y) and robot.type != 'QUADCOPTER':
            return
        
        robot.set_position(x, y)
        
       
            
    
    def plot_warehouse(self):
        plt.imshow(self.warehouse, cmap='gray', vmin=0, vmax=255)
        for robot in self.robots:
            plt.plot(robot.x, robot.y, 'ro')
        
        self.show()
        
    def plot_warehouse_with_goal(self):
        plt.imshow(self.warehouse, cmap='gray', vmin=0, vmax=255)
        for robot in self.robots:
            plt.plot(robot.x, robot.y, 'ro')
            plt.plot(robot.goal_x, robot.goal_y, 'bo')
            
        # show all grid lines
        plt.xticks(np.arange(0, self.width, 1))
        plt.yticks(np.arange(0, self.height, 1))
        plt.grid(True)    
        self.show()
        
    def add_obstacle(self, x, y):
        self.warehouse[x, y] = 0
        
    def get_obstacles(self):
        obstacles = []
        for x, y in np.ndindex(self.warehouse.shape):
            if self.warehouse[x, y] == 0:
                obstacles.append((x, y))
        return obstacles
        
        
    def plot_warehouse_with_goal_path(self):
        plt.imshow(self.warehouse, cmap='gray', vmin=0, vmax=255)
        
        for robot in self.robots:
    
            if robot.color == 'red':
                robo_pos = 'ro'
                robo_goal = 'rs'
                robo_path = 'r--'
            elif robot.color == 'green':
                robo_pos = 'go'
                robo_goal = 'gs'
                robo_path = 'g--'
            elif robot.color == 'blue':
                robo_pos = 'bo'
                robo_goal = 'bs'
                robo_path = 'b--'
            
            plt.plot(robot.x, robot.y, robo_pos)
            plt.plot(robot.goal_x, robot.goal_y, robo_goal)
            plt.plot([robot.x, robot.goal_x], [robot.y, robot.goal_y], robo_path)
            
            
        self.show()
        
    def plot_warehouse_with_goal_path_type(self):
        plt.imshow(self.warehouse, cmap='gray', vmin=0, vmax=255)
        for robot in self.robots:
            plt.plot(robot.x, robot.y, 'ro')
            plt.plot(robot.goal_x, robot.goal_y, 'bs')
            # plot path from robot to goal with dashed line
            plt.plot([robot.x, robot.goal_x], [robot.y, robot.goal_y], 'g--')
            plt.text(robot.x, robot.y, robot.type)
        self.show()
               
    def show(self):
        # show all grid lines
        plt.gca().invert_yaxis()
        plt.xticks(np.arange(0, self.width, 1))
        plt.yticks(np.arange(0, self.height, 1))
        plt.grid(True)    
        
        # create legend for robots
        handles = []
        for robot in self.robots:
            if robot.color == 'red':
                patch = plt.Line2D([0], [0], marker='o', color='w', label='Quadcopter', markerfacecolor='r', markersize=10)
            elif robot.color == 'green':
                patch = plt.Line2D([0], [0], marker='o', color='w', label='Differential Drive', markerfacecolor='g', markersize=10)
            elif robot.color == 'blue':
                patch = plt.Line2D([0], [0], marker='o', color='w', label='Humanoid', markerfacecolor='b', markersize=10)
            
            handles.append(patch)
                
        plt.legend(handles=handles, bbox_to_anchor=(1.3, .6), loc='right')
        

            
        
        plt.show()
    
    def is_position_occupied(self, x, y):
        for robot in self.robots:
            if robot.x == x and robot.y == y and robot.x != robot.goal_x and robot.y != robot.goal_y and robot.type != 'QUADCOPTER':
                return True
        return False
        
    def process_move_request(self):
        request_moves = []
        for robot in self.robots:
            move_x, move_y = robot.get_move_request()
            request_moves.append((robot, move_x, move_y))
            print(robot.type, move_x, move_y)
            
            
        # check it any moves collide with other robots
        for robot, move_x, move_y in request_moves:
            for other_robot in self.robots:
                other_move_x, other_move_y = other_robot.get_move_request()
                if robot != other_robot and move_x == other_move_x and move_y == other_move_y:
                    # if other robot is closer to its goal, then don't move
                    if len(other_robot.get_shortest_path()) > len(robot.get_shortest_path()):
                        request_moves.remove((robot, move_x, move_y))
                    else:
                        request_moves.remove((other_robot, other_move_x, other_move_y))
                
        # move robots
        for robot, move_x, move_y in request_moves:
            self.move_robot(robot, move_x, move_y)
            
            
             
            


# create warehouse
warehouse = Warehouse(10, 10)

# create robots
robot1 = Robot(1, 4, 'QUADCOPTER', warehouse)

robot2 = Robot(4, 1, 'DIFFERENTIAL_DRIVE', warehouse)

robot4 = Robot(3, 1, 'DIFFERENTIAL_DRIVE', warehouse)

robot3 = Robot(1, 3, 'HUMANOID', warehouse)

# add robots to warehouse
warehouse.add_robot(robot1)
warehouse.add_robot(robot2)
warehouse.add_robot(robot3)
warehouse.add_robot(robot4)

# set goals for robots
robot1.set_goal(9, 9)
robot2.set_goal(4, 9)
robot3.set_goal(9, 5)
robot4.set_goal(9, 3)

# plot warehouse
#warehouse.plot_warehouse_with_goal_path()


# animate robots moving to their goals
def animate(i):
    
    plt.cla()

    robot1.move_towards_goal()
    robot2.move_towards_goal()
    robot3.move_towards_goal()
    robot4.move_towards_goal()
    
    warehouse.process_move_request()
    warehouse.plot_warehouse_with_goal_path()
    
 
    if len(warehouse.robots) == 0:
        ani.event_source.stop()
        

    return ani

ani = FuncAnimation(plt.gcf(), animate, frames=12, interval=1000)
plt.show()


 
      
    


