import math
import random
import shapely
import numpy as np
import matplotlib.pyplot as plt 
import geopandas as gpd
import shapely.geometry


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

class rrt_initializer():

    def __init__(self,max_iter=50,start=[0.5,0.5],end=[11,12]):
        #this assumes the robot starts from a position of 0,0 in the bottom left corner o.e.
        self.start = start
        self.end = end
        self.max_iter = max_iter
        self.node_list = []

    def run_rrt_algo(self):
        #initialize the series of start and end nodes configurations
        initial_node = robotNode(self.start)
        initial_node.head_angle = 0
        # self.end_node = robotNode(self.end)
        self.node_list.append(initial_node)

        for i in range(self.max_iter):
            #generate a new sample node to add to list
            new_node = self.gen_random_sample()
            par_idx,angle = self.get_parent(new_node)
            new_node.angle = angle
            new_node.parent = self.node_list[par_idx]
            parent = new_node.parent

            old_x, old_y = parent.coords[0],parent.coords[1]
            new_x,new_y = new_node.coords[0],new_node.coords[1]

            p1,p2 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=0.5)
            p3,p4 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=-0.5)

            #gets approximately 2 line strings after this. 
            get_segment_1 = shapely.geometry.LineString([p1, p2])
            get_segment_2 = shapely.geometry.LineString([p3, p4])
            get_shape1 = map_rotations(new_x,new_y,angle,0.71)

            if self.check_collision(get_segment_1,get_segment_2,get_shape1,obstacles)!=True:
                self.node_list.append(new_node)
                e1,e2 = self.end[0],self.end[1]
                #check if the nodes intersect. 
                check_true = shapely.intersects(shapely.Point(e1, e2),get_shape1)
                if check_true:
                    path_list = self.return_path(new_node)
                    return path_list,"T"

        dist_idx = self.get_min_dist()
        min_dist_node = self.node_list[dist_idx]

        return self.return_path(min_dist_node),"F"
            
    def get_parent(self,new_node):
        '''
        Searches te node list space and finds the parent of the new node and 
        returns the nearest index or something equivalent.
        
        '''
        x_val,y_val = new_node.coords[0],new_node.coords[1]
        ind = 0
        min_dist = float('inf')

        #loop through the set of nodes and get the x and y coordinates. 
        for i,parent in enumerate(self.node_list):
            #isolate the parent x and y coordinates
            par_x,par_y = parent.coords[0],parent.coords[1]
            dist = (par_x-x_val)**2+(par_y-y_val)**2

            #obtain the euclidean distance metric 
            dist = math.sqrt(dist)

            if dist<min_dist:
                ind = i
                min_dist = min(dist,min_dist)
                angle = math.atan2(y_val-par_y,x_val-par_x)

        #get the node index of nearest node and the angle
        return ind,math.degrees(angle)

    def gen_random_sample(self):
        '''
        This function will generate a random sample. 
        
        '''
        # This will generate a single random sample for adding this to the values
        gen_rand1 = random.randrange(10,2500,1)/100
        gen_rand2 = random.randrange(10,2500,1)/100

        #gives a new robot node with the x and y coordinates
        return robotNode([gen_rand1,gen_rand2])

    def check_collision(self,seg1,seg2,shape1,obstacles):
        '''
        Loop through the set of obstacles and determine if they collide with the segments
        '''
        #the current function checks whether the robot represented as a square collides with an obstacle
        for entry in obstacles:

            #check intersection conditions
            if shapely.intersects(entry,seg1) or shapely.intersects(entry,seg2) or  shapely.intersects(entry,shape1):
                return False
            
        return True
    
    def return_path(self,end_node):
        '''
        Check the path nodes that are there.
        '''
        path_nodes = []

        #loop through the parent nodes and check that they are on path.
        while end_node.parent!=None:
            path_nodes.insert(0,end_node)
            end_node = end_node.parent

        return path_nodes

    def get_min_dist(self):
        '''

        This function is used to get the minimum distance between the node and the goal location. 

        '''
        min_dist = float('inf')
        min_ind = 0

        for ind,node in enumerate(self.node_list):
            #isolate the coordinates and put them as x and y values. 
            x_val,y_val = node.coords[0],node.coords[1]
            dist = (self.end[0]-x_val)**2+(self.end[1]-y_val)**2
            #obtain the euclidean distance metric 
            dist = math.sqrt(dist)
            if dist<min_dist:
                min_ind = ind
        
        return min_ind
        

def map_first_two_points(x1,y1,x2,y2,angle,diff=0.5):
    '''
    This function is used to map the first two points
    onto the output space
    '''

    # establish and define a series of two points
    point_1 = np.array([[-diff],[-diff]])
    point_2 = np.array([[-diff],[diff]])
    rad_ang = math.radians(angle)

    # define the relevant rotation parameters for the model
    rot_mat = np.array([[math.cos(rad_ang),-math.sin(rad_ang)],[math.sin(rad_ang),math.cos(rad_ang)]])

    # obtain the rotations of the two points to get the outputs
    rot_first = np.matmul(rot_mat,point_1)
    rot_sec = np.matmul(rot_mat,point_2)

    #gets the first and second points post rotation
    map_first = [x1+rot_first[0],y1+rot_first[1]]
    map_sec = [x2+rot_sec[0],y2+rot_sec[1]]

    return map_first,map_sec

def test_nodes_basic():
    '''

    Runs an example test to map a series of nodes from start to end destination. 

    '''
    #initializes the robot node from start
    first_node = robotNode(0.5,0.5)
    node_2 = robotNode(9,3)
    node_3 = robotNode(8,7)
    node_4 = robotNode(11,12)

    #defines a series of node parents. 
    node_2.parent = first_node
    node_3.parent = node_2
    node_4.parent = node_3

    #forms a list of all robot nodes.
    list_arr = [first_node,node_2,node_3,node_4]
    return list_arr



def initialize_objects(obs_list):
    '''

    Inputs, Expect the Params of obs_list To Be Like:
    obs_list = [[x_a,y_a,w_a],[x_b,y_b,w_b]]

    Output:
    type(list) --> represents the output values 

    '''
    render_obstacles = []

    for entry in obs_list:
        #initializes a circle on the map representing the set of points.
        make_point = shapely.Point(entry[0],entry[1]).buffer(entry[2])

        #append the given obstacle to the list of renderable obstacles
        render_obstacles.append(make_point)
    
    return render_obstacles

def map_rotations(x,y,angle,diff=0.5):
    '''
    This function defines a way of mapping the rotation values 
    between to coordinate poins and information
    
    '''
    points = [(-diff, -diff), (-diff, diff), 
              (diff, diff), (diff, -diff)]

    #define a matrix array with the coordinates
    rad_ang = math.radians(angle)

    #define the rotation points. 
    rot_arr = np.array([[math.cos(rad_ang),-math.sin(rad_ang)],[math.sin(rad_ang),math.cos(rad_ang)]])
 
    # Apply the rotation to each point
    rotated_points = [np.matmul(rot_arr, np.array(p)).tolist() for p in points]
    
    for i,ind in enumerate(rotated_points):
        sum_ind = [ind[0]+x,ind[1]+y]
        rotated_points[i]=sum_ind
    
    # Create a polygon with the rotated points
    get_polygon = shapely.Polygon(rotated_points)
    
    return get_polygon


obj_list = [[10,10,0.5],[21,21,1],[11,11,6]]

#get the mapping of the first two points by transformation.


#initialize a series of obstacles to be defined. 
obstacles = initialize_objects(obj_list)

if __name__=="__main__":
    #runs an rrt initializer and obtains the path.
    #initialize a series of obstacles to be defined. 
    obstacles = initialize_objects(obj_list)
    rrt = rrt_initializer()
    obtained_path = rrt.run_rrt_algo()


