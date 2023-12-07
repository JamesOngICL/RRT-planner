import shapely
import geopandas as gpd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
from shapely import Polygon
import shapely.geometry


#get the x1, y1 and x2, y2 points 
x1_point,y1_point = 1,2
x2_point,y2_point = 3,8

def map_first_two_points(x1,y1,x2,y2,angle,diff=0.5):
    '''
    This function is used to map the first two points
    onto the output space

    Args: 
    x1,y1,x2,y2,angle
    '''

    # establish and define a series of two points
    if diff>0:
        point_1 = np.array([[-diff],[-diff]])
        point_2 = np.array([[-diff],[diff]])
    elif diff<0:
        point_1 = np.array([[-diff],[diff]])
        point_2 = np.array([[-diff],[-diff]])
    
    #have a way of mapping angles between degrees and radians
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

#get the mapping of the first two points by transformation.
p1, p2 = map_first_two_points(x1_point,y1_point,x2_point,y2_point,0,diff=0.5)
p3, p4 = map_first_two_points(x1_point,y1_point,x2_point,y2_point,0,diff=-0.5)

print(p1,p3)

#gets a line segment. 
get_segment_1 = shapely.geometry.LineString([p1, p2])
get_segment_2 = shapely.geometry.LineString([p3, p4])
circle = shapely.Point(2, 3).buffer(0.6)

#get the series of values
seg_series = gpd.GeoSeries([get_segment_1,get_segment_2,circle])

#plot the first and second geoseries values
seg_series.plot(alpha=0.5)
my_intersection = shapely.intersects(circle,get_segment_1)
print(my_intersection)

my_intersection2 = shapely.intersects(circle,get_segment_2)
print(my_intersection2)

#show the values
plt.show()

'''

fig = plt.figure() 
ax = fig.add_subplot(111) 

#essentially just give the bottom coordinate and width height parameters which are needed for the plotting and execution. 
#the left corner is essentially -diff,-diff which can be used. 
rect1 = matplotlib.patches.Rectangle((-200, -100), 
                                     400, 200, angle=50,
                                     color ='green') 

#go from xy[0] bottom coordinate value to xy[0] +width and xy[1]+height,
#can see that the pink rectangle is slightly shorter in height. 
rect2 = matplotlib.patches.Rectangle((0, 150), 
                                     300, 20, 
                                     color ='pink') 

#this renders a yellow rectangle which is visible on axis and plot. 
rect3 = matplotlib.patches.Rectangle((-300, -50), 
                                     40, 200, 
                                     color ='yellow') 

#plots a circle. 
circle_1 = matplotlib.patches.Circle((-150,-120), radius=30,color="black")

ax.add_patch(circle_1)
ax.add_patch(rect1) 
ax.add_patch(rect2) 
ax.add_patch(rect3) 

#define the x and y limit coordinates
plt.xlim([-400, 400]) 
plt.ylim([-400, 400]) 
  
plt.show() 

'''

'''
def test_shapes():
    poly1 = shapely.Polygon([(0, 0), (1,0), (1,1), (0,1)] )
    poly2 = shapely.Polygon( [(0.25, 0.25), (0.5,0.25), (0.5,0.5), (0.25,0.5) ] )
    poly3 = shapely.Polygon([(5,5),(5,6),(6,5),(6,6)])
    circle = shapely.Point(5, 5).buffer(3)

    #get a circle series. 
    circle_series = gpd.GeoSeries([circle])

    #checks whether two points interesect in shapely
    get_diff = shapely.intersects(circle,poly3)

    #false, false, true as expected. 
    print(shapely.intersects(poly1,poly3))
    print(shapely.intersects(circle,poly2))

    print(get_diff)

    circle_series.plot()
    plt.show()

    polydiff = poly1.difference(poly2)
    get_geom_diff0 = shapely.intersection(poly1,poly3)

    #checks the intersection between two polygons present. 
    get_geom_diff = shapely.intersection(poly1,poly2)
    myPoly = gpd.GeoSeries([polydiff])
    new_geo_diff = shapely.intersection(circle,poly3)

    print(new_geo_diff.area)

    # myPoly.plot()
    new_series = gpd.GeoSeries([poly2])

    print(get_geom_diff)
    print(get_geom_diff0)
    print(get_geom_diff0.area)


    #the code for a new shape intersecting will involve this. 
    if get_geom_diff0.area==0:
        print("Legal Instruction Executed")

    if get_geom_diff.area==0:
        print("Executed Illegal Instruction!")

def map_rotations(x,y,angle,diff):

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

# print(map_rotations(45,1))

polygon = map_rotations(3,3,45,1)

shape_series = gpd.GeoSeries([polygon])
# map_envelope = shape_series.envelope
fig = plt.figure() 
ax = fig.add_subplot(111) 


#prints the series of points
shape_series.plot(ax=ax, color='blue',alpha=0.6)
circle = shapely.Point(5, 5).buffer(2.5)

#get a circle series. 
circle_series = gpd.GeoSeries([circle])
circle_series.plot(ax=ax, color='red', alpha=0.6)
#the shapely intersects function works like gold or is amazing. 
#I think its time to go back and continue this code. Today has been an excellent day. 
print(shapely.intersects(circle_series,shape_series))

plt.show()
# test_shapes()
#this will plot the single shape
# new_series.plot()
# plt.show()

'''