#!/usr/bin/env python
import roslib
roslib.load_manifest('multi_turtlebot_opt')
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def handle_turtle_pose(msg, turtlename):
    point = msg.pose.pose.position
    quaternion = msg.pose.pose.orientation
    br = tf.TransformBroadcaster()
    br.sendTransform((point.x, point.y, 0),
                     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                      rospy.Time.now(),
                      turtlename,
                      "world")

    #print("turtlename: "+ turtlename +" qx: " + bytes(quaternion.x) + " qy: " + bytes(quaternion.y) + " qz: " + bytes(quaternion.z) + " qw: " + bytes(quaternion.w))

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcunch aster')
    turtlename = rospy.get_param('~turtle')
    print 'turtlename: ' + turtlename
    rospy.Subscriber('/%s/odom' % turtlename,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
    
    rospy.spin()
