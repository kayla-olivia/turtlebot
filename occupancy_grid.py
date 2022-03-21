import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

#initialize node
rospy.init_node('later_to_occupancy_grid_node')
robot_pose = tf.TransformListener

#initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'Map'
resolution = 0.01
width = 500
height = 500

#initialize robot pose relative to world
x_robot = 0.0
y_robot = 0.0

#square size of the robot footprint
footprint = 0.1

#map update rate 5 Hz
rate 5.0

#range data
robot_range = 0.0

def set_free_cells(grid, position, size):
    
    global resolution
    off_x = position[1] // resolution + width // 2
    off_y = position[0] // resolution + height // 2

    for i in range(-size//2, size//2):
        for j in range(-size//2,size//2):
            grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid,position,orientation,position_sonar,quaternion_sonar,car_range):
    
    global resolution
    off_x = position[1] // resolution + width // 2
    off_y = position[0] // resolution + height // 2
    
    euler = tf.transform.euler_from_quaternion(orientation)

    if not robot_range == 0.0:
        rotMatrix = numpy.array([[numpy.cos(euler[2]),numpy.sin(euler[2])],[-numpy.sin(euler[2]),numpy.cos(euler[2])]])
        obstacle = numpy.dot(rotMatrix,numpy.array([0,(robot_range + position)]))
        rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0],obstacle[1])

        #set probability of occupancy to 100 and neighbor cells to 50
        grid[int(obstacle[0]),int(obstacle[1])] = int(100)
        if grid[int(obstacle[0]+1), int(obstacle[1])] < int(1):
            grid[int(obstacle[0]+1), int(obstacle[1])] = int(50)

        if grid[int(obstacle[0]), int(obstacle[1]+1)] < int(1):
            grid[int(obstacle[0]), int(obstacle[1]+1)] = int(50)
        
        if grid[int(obstacle[0]-1), int(obstacle[1])] < int(1):
            grid[int(obstacle[0]-1), int(obstacle[1])] = int(50)

        if grid[int(obstacle[0]), int(obstacle[1]-1)] < int(1):
            grid[int(obstacle[0]), int(obstacle[1]-1)] = int(50)
        
        t = 0.5
        i = 1
        free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
        while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
            grid[int(free_cell[0]),int(free_cell[1])] = int(0)
            free_cell = numpy.dot(rotMatrix,numpy.array([0,t*i])) + numpy.array([off_x,off_y])
            i = i+1

def callback_range(msg):
    global robot_range
    robot_range = msg.ranges[0]

#subscribers
range_sub = rospy.Subscriber("/robot/scan", LaserScan, callback_range)
#publishers
occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)

#main function
if __name__ == '__main__':

    #setting grid parameters
    if rospy.has_param("occupancy_rate"):
        rate = rospy.get_param("occupancy_rate")

    if rospy.has_param("grid_resolution"):
        resolution = rospy.get_param("grid_resolution")

    if rospy.has_param("grid_width"):
        width = rospy.get_param("grid_width")
    
    if rospy.has_param("grid_height"):
        height = rospy.get_param("grid_height")

map_msg.info.resolution = resolution
map_msg.info.width = width
map_msg.info.height = height
map_msg.data = range(width*height)

#initialize grid with -1 (unknown)
grid = numpy.ndarray((width,height),buffer=numpy.zeros((width,height),dtype=numpy.int),dtype=numpy.int)
grid.fill(int(-1))

#set map origin [meters]
map_msg.info.origin.position.x = - width // 2 * resolution
map_msg.info.origin.position.y = - height // 2 * resolution

loop_rate = rospy.Rate(rate)

while not rospy.is_shutdown():
    try:
        t = robot_pose.getLatestCommonTime("/robot_base_link","/world")
        position, quaternion = robot_pose.lookupTransform("/world","/robot_base_link",t)
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

set_free_cells(grid,position,int(footprint//resolution))
set_obstacle(grid,position,quaternion,position_sonar,quaternion_sonar,robot_range)
map_msg.header.stamp = rospy.Time.now()

for i in range(width*height):
    map_msg.data[i] = grid.flat[i]
occ_pub.publish(map_msg)

loop_rate.sleep()