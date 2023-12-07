import math
import random
import shapely
import numpy as np
import matplotlib.pyplot as plt 
import geopandas as gpd
import shapely.geometry
import time
import csv
import pygame
# Averages: [145,340,421,470,476]

#define the window width and height params. 
py_width,py_height = 600,600
max_width,max_height = 2500,2500

def get_dist(path_list):
    '''
    This function returns the distance from source to end node 

    Args:
    type(list) --> path_list containing all objects on path.

    Outputs:
    type(float) --> dist

    '''
    curr_dist = 0

    for i in range(1,len(path_list)):
        prev_node = path_list[i-1]
        curr_coords = path_list[i].coords
        c_x,c_y = curr_coords[0],curr_coords[1]
        p_x,p_y = prev_node.coords[0],prev_node.coords[1]
        dist = (c_x-p_x)**2+(c_y-p_y)**2
        dist = math.sqrt(dist)
        curr_dist += dist
    
    return curr_dist


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

    def __init__(self,max_iter=50,start=[0.5,0.5],end=[19,17]):
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
            rand_sample = self.gen_random_sample()
            par_idx,_ = self.get_parent(rand_sample)
            parent = self.node_list[par_idx]

            #get the coordinates from the parent node 
            new_coords = self.get_best_sample(parent)
            new_node = robotNode(new_coords)
            par_idx,angle = self.get_parent(new_node)
            new_node.angle = angle
            new_node.parent = parent

            #form a shape to determine whether intersections occur. 
            new_shape1 = map_rotations(new_coords[0],new_coords[1],angle,0.71)
            e1,e2 = self.end[0],self.end[1]

            #check if the nodes intersect. 
            self.node_list.append(new_node)
            check_true = shapely.intersects(shapely.Point(e1, e2),new_shape1)

            #ensures that we are minimally distant to the node
            if check_true:
                path_list = self.return_path(new_node)
                # dist = get_dist(path_list)
                return path_list,"T"

        # Get the minimal distance
        dist_idx = self.get_min_dist()

        min_dist_node = self.node_list[dist_idx]
        path_list = self.return_path(min_dist_node)

        # dist = get_dist(path_list)

        return self.return_path(min_dist_node),"F"

    def get_best_sample(self,parent):
        '''
        Runs an RRT function example to enable a better sample to be chosen

        Args:
        type(robotNode) --> parent

        Output:
        type(list) --> new_coords

        '''
        curr_coords = parent.coords
        goal = self.end
        g_x, g_y = goal[0],goal[1]
        min_dist = float('inf')
        new_coords = []
        samples = 0

        while samples<10:

            #get a series of distance and angle values
            dist = random.randrange(2,80)/10
            angle = 2*math.pi*random.randrange(0,360)/180
            x_shift,y_shift = dist*math.cos(angle),dist*math.sin(angle)
            #obtain mappings to new x and y coordinates. 
            new_x,new_y = curr_coords[0]+x_shift,curr_coords[1]+y_shift

            if new_x<0 or new_y<0:
                continue

            old_x, old_y = parent.coords[0],parent.coords[1]
            # new_x,new_y = new_node.coords[0],new_node.coords[1]

            p1,p2 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=0.5)
            p3,p4 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=-0.5)

            #obtain a distance to the goal 
            goal_dist = (new_x-g_x)**2+(new_y-g_y)**2
            goal_dist = math.sqrt(goal_dist)

            #gets approximately 2 line strings after this. 
            get_segment_1 = shapely.geometry.LineString([p1, p2])
            get_segment_2 = shapely.geometry.LineString([p3, p4])
            get_shape1 = map_rotations(new_x,new_y,angle,0.71)

            collision_status = self.check_no_collision(get_segment_1,get_segment_2,get_shape1,obstacles)
            #obtain the new sample. 
            if collision_status==True:
                samples += 1

                if min_dist>goal_dist :
                    new_coords = [new_x,new_y]
                    min_dist = goal_dist

        return new_coords

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
        gen_rand1 = random.randrange(10,max_width,1)/100
        gen_rand2 = random.randrange(10,max_height,1)/100

        #gives a new robot node with the x and y coordinates
        return robotNode([gen_rand1,gen_rand2])

    def check_no_collision(self,seg1,seg2,shape1,obstacles):
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
        tot_dist = 0
        #loop through the parent nodes and check that they are on path.
        while end_node.parent!=None:
            #obtain the distance between the nodes at each iteration
            x_dist = (end_node.parent.coords[0]-end_node.coords[0])
            y_dist = (end_node.parent.coords[1]-end_node.coords[1])
            tot_dist += math.sqrt(x_dist**2+y_dist**2)

            path_nodes.insert(0,end_node)
            end_node = end_node.parent

        path_nodes.insert(0,end_node)

        return path_nodes,tot_dist

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
                min_dist = dist
        
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


obj_list = [[6,10,0.5],[21,21,1],[14,17,3],[3,5,2]]

#get the mapping of the first two points by transformation.

#initialize a series of obstacles to be defined. 
obstacles = initialize_objects(obj_list)

def plot_circles(screen,colours):
    '''
    This function is used to plot circles
    '''
    for entry in obj_list:
        cx,cy = entry[0]*(py_width/max_width)*100,entry[1]*(py_height/max_height)*100

        r_val = entry[2]*(py_height/max_height)*100

        pygame.draw.circle(screen,color=colours["GREEN"],center=(cx,cy),radius=r_val)
        pygame.display.update()

    time.sleep(1)

def get_main_nodes():
    '''
    Runs the RRT algorithm and gets the path from source to dest. 
    '''
    rrt = rrt_initializer()
    obtained_path,cond = rrt.run_rrt_algo()
    return obtained_path[0],obtained_path[1],cond


if __name__=="__main__":
    #runs an rrt initializer and obtains the path.
    #initialize a series of obstacles to be defined.

    # screen = pygame_start()
    # plot_list_nodes(list_arr,screen)
    # obstacles = initialize_objects(obj_list)
    # rrt = rrt_initializer()
    # obtained_path = rrt.run_rrt_algo()
    # field names  
    fields = ['Path_len', 'Reached']
    # name of csv file  
    filename = "50iter.csv"
    
    rows = []

    for i in range(500):
        plist,dist,cond = get_main_nodes()
        rows.append([dist,cond])
        if i%50==0:
            print("ITERATION ",i," VALUES:",dist,cond)
    
    # writing to csv file  
    with open(filename, 'w') as csvfile:  
        # creating a csv writer object  
        csvwriter = csv.writer(csvfile, dialect='unix')  
            
        # writing the fields  
        csvwriter.writerow(fields)  
            
        # writing the data rows  
        csvwriter.writerows(rows) 
