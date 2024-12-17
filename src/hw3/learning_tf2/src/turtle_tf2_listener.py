#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

def create_following(followers, tfBuffer, velocities):
    for follower, carrot, turtle_vel in followers:
        try:
            trans = tfBuffer.lookup_transform(follower, carrot, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not transform {follower} to {carrot}: {e}")
            continue

        angular_speed = rospy.get_param('~angular_speed', 4.0)
        linear_speed = rospy.get_param('~linear_speed', 0.5)
        
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = angular_speed * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = linear_speed * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        
        turtle_vel.publish(msg)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)

    rospy.loginfo("Spawning turtle2 and turtle3")
    spawner(4, 2, 0, 'turtle2')
    spawner(6, 2, 0, 'turtle3')

    turtle2_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    turtle3_vel = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    # List of (follower, carrot, publisher) tuples
    followers = [
        ('turtle2', 'turtle1', turtle2_vel),
        ('turtle3', 'turtle2', turtle3_vel)
    ]

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        create_following(followers, tfBuffer, [turtle2_vel, turtle3_vel])
        rate.sleep()

    rospy.loginfo("Shutting down tf2_turtle_listener node")
