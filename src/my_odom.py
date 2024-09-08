#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None 
        self.dist = 0.0
        self.yaw = 0.0
        self.publish_data_count = 0
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose: Pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        if not self.old_pose:
            self.old_pose = cur_pose.position
            return
        
        x_diff = cur_pose.position.x - self.old_pose.x
        y_diff = cur_pose.position.y - self.old_pose.y


        distance = (x_diff ** 2 + y_diff ** 2) ** (0.5)
        self.dist += distance
        self.old_pose = cur_pose.position

    def update_yaw(self, cur_orientation: Quaternion):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        # Extract the quaternion (x, y, z, w) from the input orientation
        orientation_q = cur_orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Update the yaw (rotation around the z-axis) in radians
        self.yaw = yaw



    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.

        data = Point()

        # Assign the distance to the x field and yaw to the y field of the Point object
        data.x = self.dist  # Total distance moved
        data.y = self.yaw   # Current yaw angle (in radians)

        # The z field is unused
        data.z = 0.0

        self.my_odom_pub.publish(data)
        if self.publish_data_count % 20 == 0:
            rospy.loginfo(f"dist: {self.dist:.4f}, yaw: {self.yaw:.4f}")
        self.publish_data_count += 1
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
