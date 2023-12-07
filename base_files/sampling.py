import random
import math

class robotNode():

    def __init__(self,coords):
        '''
        The primary purpose of this robotNode is to 

        Red = Obstacle, Blue = Line, Black = robot.
        '''
        #store the coordinates in X,Y value array
        self.coords = coords
        #initially have the robot angle as being none
        #might need to use the atan2 parameter
        self.head_angle = None

        #define the parent nodes and x,y paths. 
        self.parent = None

        #store the path taken to get to the current node at present (like djikstra backtracing)
        self.xsteps = []
        self.ysteps = []


def get_closest_sample(node,goal):
    '''
    Function generates parent samples for distances and angles. 
    '''
    #generates pairs of angles and distance based parameters. 
    curr_coords = node.coords
    g_x, g_y = goal[0],goal[1]
    min_dist = float('inf')
    new_coords = []

    for i in range(10):
        #get a series of distance and angle values
        dist = random.randrange(2,50)/10
        angle = 2*math.pi*random.randrange(0,360)/180
        x_shift,y_shift = dist*math.cos(angle),dist*math.sin(angle)
        #obtain mappings to new x and y coordinates. 
        new_x,new_y = curr_coords[0]+x_shift,curr_coords[1]+y_shift

        #obtain a distance to the goal 
        goal_dist = (new_x-g_x)**2+(new_y-g_y)**2
        goal_dist = math.sqrt(goal_dist)

        #obtain the new sample. 
        if min_dist>goal_dist:
            new_coords = [new_x,new_y]

        
start_node = [0,0]
make_node = robotNode(start_node)