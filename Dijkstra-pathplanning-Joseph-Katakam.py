## Importing Necessary Libraries
import numpy as np # To hanlde Array Operations
import matplotlib.pyplot as plt # Plotting Functionalities
import time # To check time of Execution
import heapq # Queueing Method to obtain the least value (cost) in the list


## Creating a Class for Node with it's parameters as X,Y Coordinates, Cost at each Node, it's Parent
class Node:
    
    # Initialize Data
	def __init__(self, x, y, cost, parent_node):

		self.x = x
		self.y = y
		self.cost = cost
		self.parent_node = parent_node
	
    ## To support Node Comparision
	def __lt__(self,other):
		return self.cost < other.cost

## Defining the Action Set to Move the Node

## For Orthogonal Movement the Cost is 1; i.e. Up, Down, Left and Right
def move_up(x,y,cost):
	y += 1
	cost += 1
	return x,y,cost

def move_down(x,y,cost):
	y -= 1
	cost += 1
	return x,y,cost

def move_left(x,y,cost):
	x -= 1
	cost += 1
	return x,y,cost

def move_right(x,y,cost):
	x += 1
	cost += 1
	return x,y,cost

## For Diagnal Movement the Cost is 1.4; i.e. Up_Right, Down_Right, Up_Left and Down_Left
def move_upright(x,y,cost):
	x += 1
	y += 1
	cost += np.sqrt(2)
	return x,y,cost

def move_downright(x,y,cost):
	x += 1
	y -= 1
	cost += np.sqrt(2)
	return x,y,cost

def move_upleft(x,y,cost):
	x -= 1
	y += 1
	cost += np.sqrt(2)
	return x,y,cost

def move_downleft(x,y,cost):
	x -= 1
	y -= 1
	cost += np.sqrt(2)
	return x,y,cost

## Moving the Node in accordance with the Action
def move_node(move,x,y,cost):
    
    if move == 'Up':
        return move_up(x,y,cost)
    elif move == 'UpRight':
        return move_upright(x,y,cost)
    elif move == 'Right':
        return move_right(x,y,cost)
    elif move == 'DownRight':
        return move_downright(x,y,cost)
    elif move == 'Down':
        return move_down(x,y,cost)
    elif move == 'DownLeft':
        return move_downleft(x,y,cost)
    elif move == 'Left':
        return move_left(x,y,cost)
    elif move == 'UpLeft':
        return move_upleft(x,y,cost)
    else:
        return None

## Plotting the Robot Space and the Obstacles in it. Also storing the coordinates of Obstacle Space.
def plot_map( width, height):
    
    # Generating Obstacle Space
    obstacle_space = np.full((height, width),0)
    
    radius = 40 # For the Circular Obstacle
    for y in range(0, height) :
        for x in range(0, width):
            
            ## Plotting Clearance Space for the Obstacles using Half Plane Equations
            
            ## Polygon Obstacle
            p1 = (y-5) - ((0.316) *(x+5)) - 173.608  
            p2 = (y+5) + (1.23 * (x+5)) - 229.34 
            p3 = (y-5) + (3.2 * (x-5)) - 436 
            p4 = (y+5) - 0.857*(x-5) - 111.42 
            p5 = y + (0.1136*x) - 189.09
            
            ## Circular Obstacle
            cir_clearance = ((y -185)**2) + ((x-300)**2) - (radius+5)**2  
            
            ## Hexagon Obstacle
            h1_clr = (y-5) - 0.577*(x+5) - 24.97
            h2_clr = (y-5) + 0.577*(x-5) - 255.82
            h3_clr = (x-6.5) - 235
            h4_clr = (y+5) - 0.577*(x-5) + 55.82
            h5_clr = (y+5) + 0.577*(x+5) - 175 
            h6_clr = (x+6.5) - 165  
            
            ## Line Constraints to plot Clearance of Obstacles
            if(h1_clr<0 and h2_clr<0 and h3_clr<0 and h4_clr>0 and h5_clr>0 and h6_clr>0) or cir_clearance<=0  or (p1<0 and p5>0 and p4>0)or (p2>0 and p5<0 and p3<0):
                obstacle_space[y, x] = 1
             
            ## Plotting Actual Object Space Half Plane Equations
            
            ## Polygon Obstacle
            l1 = y - ((0.316) *x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857*x - 111.42 
            l5 = y + (0.1136*x) - 189.09
            
            ## Circular Obstacle
            cir_actual = ((y -185)**2) + ((x-300)**2) - (radius)**2 
            
            ## Hexagon Obstacle    
            h1_real = y - 0.577*x - 24.97 
            h2_real = y + 0.577*x - 255.82
            h3_real = x - 235
            h4_real = y - 0.577*x + 55.82
            h5_real = y + 0.577*x - 175 
            h6_real = x - 165 
            
            ## Line Constraints to plot Real Obstacles
            if(h1_real<0 and h2_real<0 and h3_real<0 and h4_real>0 and h5_real>0 and h6_real>0) or cir_actual < 0 or (l1<0 and l5>0 and l4>0)or (l2>0 and l5<0 and l3<0):
                obstacle_space[y, x] = 2

    return obstacle_space

## Check if the Goal Node is Reached
def check_goal(current, goal):

	if (current.x == goal.x) and (current.y == goal.y):
		return True
	else:
		return False

## Check if the Move made is a Valid Move
def check_valid(x, y, obstacle_space):
    
    # Map Limits
	size = obstacle_space.shape
    
    # Check the Boundary Criteria
	if( x > size[1] or x < 0 or y > size[0] or y < 0 ):
		return False
    
	# Check if in Obstacle Space
	else:
		try:
			if(obstacle_space[y][x] == 1) or (obstacle_space[y][x] == 2):
				return False
		except:
			pass
	return True

# Function for Generating a Unique ID
def unique_id(node):
    id = 2122*node.x + 113*node.y 
    return id


## Dijkstra Algorithm
def dijkstra_algorithm(start, goal, obstacle_space):
    
    ## Check if given Node is the Goal Node
    if check_goal(start, goal):
        return None, 1
    
    # Storing as Nodes only when Start is not Goal
    goal_node = goal
    start_node = start
    
    ## Data Handlers
    unexplored_nodes = {}
    all_nodes = []
    explored_nodes = {} 
    priority_list = []
    
    # All possible Movements for a Node
    possible_moves = ['Up','UpRight','Right','DownRight','Down','DownLeft','Left','UpLeft']
        
    #Assigning Unique Key to Start Node
    start_key = unique_id(start_node)
    unexplored_nodes[(start_key)] = start_node
    
    # Push the Start Node into the Heap Queue
    heapq.heappush(priority_list, [start_node.cost, start_node])
    
    # Loops until all the Nodes have been Explored, i.e. until priority_list becomes empty
    while (len(priority_list) != 0):

        present_node = (heapq.heappop(priority_list))[1]
        all_nodes.append([present_node.x, present_node.y])
        present_id = unique_id(present_node)

        if check_goal(present_node, goal_node):
            goal_node.parent_node = present_node.parent_node
            goal_node.cost = present_node.cost
            print("Goal Node found")
            return all_nodes,1

        if present_id in explored_nodes:
            continue
        else:
            explored_nodes[present_id] = present_node
		
        del unexplored_nodes[present_id]
        
        # Looping through all possible nodes obtained through movements
        for move in possible_moves:
            
            # New x,y,cost after moving.
            x,y,cost = move_node(move,present_node.x,present_node.y,present_node.cost)
            
            # Updating Node with the New Values
            new_node = Node(x,y,cost,present_node)
            
            # New ID
            new_node_id = unique_id(new_node)
            
            # Checking Validity
            if not check_valid(new_node.x, new_node.y, obstacle_space):
                continue
            elif new_node_id in explored_nodes:
                continue
            
            # Updating Cost and Parent, if lesser cost is found.
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_node = new_node.parent_node
            else:
                unexplored_nodes[new_node_id] = new_node
   			
            ## Pushing all the open nodes in Heap Queue to get the least costing node.
            heapq.heappush(priority_list, [ new_node.cost, new_node])
   
    return  all_nodes, 0

## Function to Back Track position from Goal Node to Start Node.
def back_track_path(goal_node):  
    
    ## Creating Lists to hold the Back tracked values from Goal to Start Node
	x_path = []
	y_path = []
    
    ## Storing the Goal Coordinates
	x_path.append(goal_node.x)
	y_path.append(goal_node.y)
    
    ## Backtrack to the preceeding Node
	parent = goal_node.parent_node
    
    ## Backtrack until Start Node is Reached (which has a parent value -1)
	while parent != -1:
        
        ## Append the Coordinates
		x_path.append(parent.x)
		y_path.append(parent.y)
        
        ## Backtrack to preceeding Node
		parent = parent.parent_node
	
    # Since the Appending was done from Goal to Start Node, Reversing them to get path from Start to Goal
	x_path.reverse()
	y_path.reverse()
    
	return x_path, y_path

## Function to Plot the Planned Path
def path_plot(start_node, goal_node, x_path, y_path, explored_nodes, obstacle_space):
    
    ## Plot the Start and Goal Positions
    plt.plot(start_node.x, start_node.y, "Db")
    plt.plot(goal_node.x, goal_node.y, "Dg")
    
    ## Plot Map
    plt.imshow(obstacle_space, "seismic")
    ax = plt.gca()
    ax.invert_yaxis()

    ## Plotting the Explored Nodes
    for i in range(len(explored_nodes)):
        plt.plot(explored_nodes[i][0], explored_nodes[i][1], "3y")
        #plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)

    plt.plot(x_path, y_path,"--r")
    plt.show()
    plt.pause(3)
    plt.close('all')

# Main Function to Execute the Program
if __name__ == '__main__':
    
    # Robot Space width and Height
    width = 400
    height = 250
    
    # Getting the Coordinates of Obstacles
    obstacle_space = plot_map(width, height)
    
    # Getting the Start and Goal Coordinates from the User    
    start_x = int(input("Enter Start Position's X Coordinate: "))
    start_y = int(input("Enter Start Position's Y Coordinate: "))

    goal_x = int(input("Enter Goal Position's X Coordinate: "))
    goal_y = int(input("Enter Goal Position's Y Coordinate: "))
    
    ## Start Timer to Check the Time of Execution
    start_time = time.time()
    
    # Validating the Start Node if it is in Permitted Robot Space
    if not check_valid(start_x, start_y, obstacle_space):
        print("Enter Start Node within the permitted Robot Space")
        exit(-1)
    
    # Validating the Goal Node if it is in Permitted Robot Space
    if not check_valid(goal_x, goal_y, obstacle_space):
        print("Enter Goal Node within the permitted Robot Space")
        exit(-1)
    
	## Creating Start Node which has attributes start (x,y) coordinates, cost = 0 and parent_node = -1
    start_node = Node(start_x, start_y, 0.0, -1)
    
    ## Creating Goal Node which has attributes goal (x,y) coordinates, cost = 0 and parent_node = -1
    goal_node = Node(goal_x, goal_y, 0.0, -1)
    
    # Implementing Dijkstra Algorithm
    explored_nodes, goal_status = dijkstra_algorithm(start_node, goal_node, obstacle_space)
    
    # Back Track only if the Goal Node is reached.
    if (goal_status == 1):
        x_path, y_path = back_track_path(goal_node)
        
        ## Plotting the Map, Obstacles and the Generated Path
        path_plot(start_node, goal_node, x_path, y_path, explored_nodes, obstacle_space)
    else:
        print("Goal could not be planned for the given points")

    ## Stopping the Timer after Program Execution
    end_time = time.time()
    print(f"Time Taken to Execute the Program is: {end_time - start_time}")
	
	



	









