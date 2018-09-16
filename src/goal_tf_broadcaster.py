#!/usr/bin/env python
import roslib
roslib.load_manifest('multi_turtlebot_opt')
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

goal_x = 0
goal_y = 0

def handle_goal_pose(goal_id):

    br = tf.TransformBroadcaster()
    br.sendTransform((goal_x, goal_y, 0),
                     (0.0, 0.0, 0.0, 1.0),
                      rospy.Time.now(),
                      goal_id,
                      "world")


if __name__ == '__main__':

    rospy.init_node('goal_tf_broadcaster')
    goal_id = rospy.get_param('~goal_id')
    goal_x = rospy.get_param('~goal_x')
    goal_y = rospy.get_param('~goal_y')
    
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        handle_goal_pose(goal_id)
