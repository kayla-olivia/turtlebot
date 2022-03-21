import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan, Range

rospy.init_node('sonar_to_laser_node')

# buffer
b1 = 0.0
b2 = 0.0
b3 = 0.0

pub = rospy.Publisher('robot/scan',LaserScan,queue_size=10)
pub_range = rospy.Publisher('robot/range',Range,queue_size=10)

Scan_msg = LaserScan()
Scan_msg.header.frame_id = "sonar_link"
Scan_msg.angle_min = math.pi/4 - 0.05
Scan_msg.angle_max = math.pi/4 + 0.05
Scan_msg.angle_increment = 0.1
Scan_msg.time_increment = 0.0
Scan_msg.range_min = 0.05
Scan_msg.range_max = 0.5

Range_msg = Range()
Range_msg.header.frame_id = "sonar_link"
Range_msg.radiation_type = 0
Range_msg.min_range = 0.05
Range_msg.max_range = 0.60
Range_msg.field_of_view = 0.17 #10 deg

def callback(msg):

    global b1,b2,b3
    b1 = b2
    b2 = b3
    b3 = msg.z / 100

    if b1 > 0.0 and b2 > 0.0 and b3 > 0.0:

        Scan_msg.ranges = [msg.z / 100]
        Range_msg.range = msg.z / 100
        Scan_msg.header.stamp = rospy.Time.now()
        Range_msg.header.stamp = rospy.Time.now()
        pub.publish(Scan_msg)
        pub_range.publish(Range_msg)

    else:
        Scan_msg.ranges = [0.0]
        Range_msg.range = 0.0
        Scan_msg.header.stamp = rospy.Time.now()
        Range_msg.header.stamp = rospy.Time.now()
        pub.publish(Scan_msg)
        pub_range.publish(Range_msg)

sub = rospy.Subscriber('robot/power',Vector3, callback)

rospy.spin()