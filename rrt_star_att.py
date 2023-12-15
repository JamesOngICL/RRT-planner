import math
import random
import shapely
import numpy as np
import matplotlib.pyplot as plt 
import geopandas as gpd
import shapely.geometry
import pygame
import time
import copy

#define the window width and height params. 
py_width,py_height = 750,750
max_width,max_height = 2500,2500
goal=[19,17]

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
        self.dist_travelled = 0

        #define the parent nodes and x,y paths. 
        self.parent = None

        #store the path taken to get to the current node at present (like djikstra backtracing)
        self.xsteps = []
        self.ysteps = []

class rrt_initializer():

    def __init__(self,max_iter=50,start=[0.5,0.5],end=goal,rad=10):
        #this assumes the robot starts from a position of 0,0 in the bottom left corner o.e.
        self.start = start
        self.end = end
        self.max_iter = max_iter
        self.node_list = []
        self.rad = rad
        self.reached = []

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

            #get the distances to parent nodes
            par_x, par_y, new_x,new_y = parent.coords[0],parent.coords[1],new_node.coords[0],new_node.coords[1]

            dist = math.sqrt((par_x-new_x)**2+(par_y-new_y)**2)
            new_node.dist_travelled = dist+parent.dist_travelled
            self.rewire_tree(new_node)
            #form a shape to determine whether intersections occur. 
            new_shape1 = map_rotations(new_coords[0],new_coords[1],angle,0.71)
            e1,e2 = self.end[0],self.end[1]

            #check if the nodes intersect. 
            self.node_list.append(new_node)
            check_true = shapely.intersects(shapely.Point(e1, e2),new_shape1)

            #ensures that we are minimally distant to the node
            if check_true:
                self.reached.append(new_node)

        #calculate the distances to nodes.
        min_dist = float('inf')
        min_ind = 10000
        if len(self.reached)>0:
            for i,entry in enumerate(self.reached):
                get_dist = entry.dist_travelled
                if get_dist<min_dist:
                    min_ind = i
                    min_dist = get_dist
            
            best_node = self.reached[min_ind]
            return self.return_path(best_node),"T",best_node.dist_travelled
        



        dist_idx = self.get_min_dist()
        min_dist_node = self.node_list[dist_idx]

        return self.return_path(min_dist_node),"F",min_dist_node.dist_travelled

    def rewire_tree(self,new_node):

        #check if this could be a parent
        new_x, new_y = new_node.coords[0],new_node.coords[1]     
        curr_dist = new_node.dist_travelled


        for entry in self.node_list:
            x,y = entry.coords[0],entry.coords[1]
            dist = math.sqrt((x-new_x)**2+(y-new_y)**2)
            mod_dist = curr_dist + dist
            #check if rewiring needs to be done
            if dist<=self.rad and mod_dist<entry.dist_travelled:
                angle = math.atan2(new_y-y,new_x-x)*180/math.pi

                p1,p2 = map_first_two_points(new_x,new_y,x,y,0,diff=0.5)
                p3,p4 = map_first_two_points(new_x,new_y,x,y,0,diff=-0.5)

                #gets approximately 2 line strings after this. 
                get_segment_1 = shapely.geometry.LineString([p1, p2])
                get_segment_2 = shapely.geometry.LineString([p3, p4])
                get_shape1 = map_rotations(new_x,new_y,angle,0.71)

                collision_status = self.check_no_collision(get_segment_1,get_segment_2,get_shape1,obstacles)

                if collision_status==True:
                    entry.parent = new_node

        return
                
            




    def get_best_sample(self,parent):
            '''
            Runs an RRT function example to enable a better sample to be chosen

            Args:
            type(robotNode) --> parent

            Output:
            type(list) --> new_coords

            '''
            curr_coords = parent.coords
            g_x, g_y = goal[0],goal[1]
            min_dist = float('inf')
            new_coords = []
            samples = 0

            while samples<10:

                #get a series of distance and angle values
                dist = random.randrange(2,80)/10
                angle = math.pi*random.randrange(0,360)/180
                x_shift,y_shift = dist*math.cos(angle),dist*math.sin(angle)
                #obtain mappings to new x and y coordinates. 
                new_x,new_y = curr_coords[0]+x_shift,curr_coords[1]+y_shift

                if new_x<0 or new_y<0 or new_x>25 or new_y>25:
                    continue

                old_x, old_y = parent.coords[0],parent.coords[1]
                # new_x,new_y = new_node.coords[0],new_node.coords[1]


                #obtain a distance to the goal 
                goal_dist = (new_x-g_x)**2+(new_y-g_y)**2
                goal_dist = math.sqrt(goal_dist)

                p1,p2 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=0.5)
                p3,p4 = map_first_two_points(new_x,new_y,old_x,old_y,0,diff=-0.5)

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

        #loop through the parent nodes and check that they are on path.
        while end_node.parent!=None:
            path_nodes.insert(0,end_node)
            end_node = end_node.parent

        path_nodes.insert(0,end_node)

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
    obtained_path,cond,dist = rrt.run_rrt_algo()
    print("CHECK REACHED:",cond,dist)
    return obtained_path,cond


def pygame_start():
    path_vals,_ = get_main_nodes()
    pygame.init()

    #define the window and screen parameters
    colours = {"BLACK": (0, 0, 0),"CYAN": (0, 255, 0),"RED": (255, 0, 0),"WHITE": (255, 255, 255),"BLUE" : (0, 0, 255),"GREEN":(0,128,0),"YELLOW":(255,255,0)}

    screen = pygame.display.set_mode((py_width,py_height)) 

    #render the background image
    bg_img = pygame.image.load('Images/map.jpg')
    bg_img = pygame.transform.scale(bg_img,(py_width,py_height))
    screen.blit(bg_img,(0,0))
    pygame.display.update()
    time.sleep(0.35)

    plot_circles(screen,colours)
    # list_arr = test_nodes_basic()
    img_2 = pygame.image.load('Images/basketball_hoop.png')
    img_2 = pygame.transform.scale(img_2,(py_width/20,py_height/20))
    img2_cx,img2_cy = goal[0]*(py_width/max_width)*100,goal[1]*(py_height/max_height)*100
    screen.blit(img_2,(img2_cx,img2_cy))

    pygame.display.update()

    plot_list_nodes(path_vals,screen)
    pygame.display.update()

    print("Here")
    time.sleep(3)

    return screen

def test_nodes_basic():
    '''

    Runs an example test to map a series of nodes from start to end destination. 

    '''
    #initializes the robot node from start
    first_node = robotNode([0.5,0.5])
    node_2 = robotNode([9,3])
    node_3 = robotNode([8,7])
    node_4 = robotNode([11,12])

    #defines a series of node parents. 
    node_2.parent = first_node
    node_3.parent = node_2
    node_4.parent = node_3

    #forms a list of all robot nodes.
    list_arr = [first_node,node_2,node_3,node_4]
    return list_arr

def plot_list_nodes(path_list,screen):
    '''
    This has the functions of plotting all elements on the path. 
    '''
    colours = {"BLACK": (0, 0, 0),"GREEN": (0, 255, 0),"RED": (255, 0, 0),"WHITE": (255, 255, 255),"BLUE" : (0, 0, 255),"CYAN":(0,128,0),"YELLOW":(255,255,0)}

    for i,entry in enumerate(path_list):
        node_x, node_y = entry.coords[0],entry.coords[1]
        map_x, map_y = node_x*(py_width/max_width)*100, node_y*(py_height/max_height)*100

        if i>=1:
            #plot the lines between the two nodes
            par_x,par_y = entry.parent.coords[0],entry.parent.coords[1]
            map_parx, map_pary = par_x*(py_width/max_width)*100, par_y*(py_height/max_height)*100

            pygame.draw.rect(screen,color=colours["RED"],rect=pygame.Rect(map_x,map_y,map_diff,map_diff),width=8)
            pygame.draw.line(screen,color=colours["WHITE"],start_pos=(map_parx, map_pary),end_pos=(map_x, map_y),width=3)


        elif i==0:
            map_diff = 0.5*(py_width/max_width)*100
            #define 4 rect params
            # ld,lu,rd,ru = node_x-map_diff,node_x
            print("START NODE:",node_x, node_y,map_x,map_y)
            pygame.draw.rect(screen,color=colours["RED"],rect=pygame.Rect(map_x,map_y,map_diff,map_diff))

        time.sleep(1)
        pygame.display.update()
    
    return 


if __name__=="__main__":
    #runs an rrt initializer and obtains the path.
    #initialize a series of obstacles to be defined.

    screen = pygame_start()
    # plot_list_nodes(list_arr,screen)
    # obstacles = initialize_objects(obj_list)
    # rrt = rrt_initializer()
    # obtained_path = rrt.run_rrt_algo()


