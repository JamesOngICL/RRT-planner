import math
import random
import shapely
class robotNode():

    def __init__(self,coords):
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

    def __init__(self,max_iter=10,start=[0.5,0.5],end=[11,12]):
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
            self.node_list.append(new_node)
            #check collision between new_node and other obstacles. 
            #steer between the nodes and set the 
        return None
            
    def get_parent(self,new_node):
        '''
        Searches te node list space and finds the parent of the new node and 
        returns the nearest index or something equivalent.
        
        '''
        x_val,y_val = new_node.coords[0],new_node.coords[1]
        ind = 0
        min_dist = float('inf')

        for i,parent in enumerate(self.node_list):
            #isolate the parent x and y coordinates
            par_x,par_y = parent.coords[0],parent.coords[1]
            dist = (par_x-x_val)**2+(par_y-y_val)**2

            #obtain the euclidean distance metric and t
            dist = math.sqrt(dist)
            if dist<min_dist:
                ind = i
                min_dist = min(dist,min_dist)
                angle = math.atan2(x_val-par_x,y_val-par_y)
        

        #get the node index of nearest node and the angle
        return ind,math.degrees(angle)

    def gen_random_sample(self):
        '''
        This function will generate a random sample. 
        
        '''
        # This will generate a single random sample for adding this to the values
        gen_rand1 = random.randrange(10,2000,1)/100
        gen_rand2 = random.randrange(10,2000,1)/100

        #gives a new robot node with the x and y coordinates
        return robotNode([gen_rand1,gen_rand2])

    def check_collision(self,curr_node):
        #the current function checks whether the robot represented as a square collides with an obstacle
        return
    
    def map_to_square(self,x_val,y_val,angle,diff=0.5):

        #we work under the assumption that the robot/rover is 1x1 in dimension. 
        x1,y1 = [0,diff]

        # polygon_vals = shapely.Polygon([(x_val-diff,y_val),(x_val+diff,y_val),
        #                                 (x_val,y_val+diff),(x_val,y_val-diff)])

        return 
    

if __name__=="__main__":
    #runs an rrt initializer and obtains the path.
    rrt = rrt_initializer()
    rrt.run_rrt_algo()