#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion
from my_odom import MyOdom

# BasicMover
class BasicMover:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        # self.cur_yaw = None
        self.odom = MyOdom()
        self.rate = rospy.Rate(10)

    # def my_odom_cb(self, msg):
    #     """Callback function for `self.my_odom_sub`."""
    #     raise NotImplementedError

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        twist = Twist()
        yaw_tolerance = 0.002  # Small tolerance for how close to target yaw

        while not rospy.is_shutdown():
            current_yaw = self.odom.yaw

            yaw_diff = target_yaw - current_yaw
            
            # Normalize the yaw difference to be between -π and π
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_diff) < yaw_tolerance:
                rospy.loginfo(f"Reached target heading: {target_yaw:.3f} rad, Current yaw: {current_yaw:.3f} rad")
                break

            remaining_yaw = abs(yaw_diff)

            if remaining_yaw <= 0.1:
                if remaining_yaw < 0.03:
                    twist.angular.z = 0.03
                else: twist.angular.z = 0.1
            else:
                twist.angular.z = 0.3

            if yaw_diff < 0:
                twist.angular.z = -twist.angular.z

            self.cmd_vel_pub.publish(twist)
            
            self.rate.sleep()

        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)


    def maintain_yaw(self, initial_yaw, twist):
        cur_yaw = self.odom.yaw 
        tolerance = 0.005

        diff = initial_yaw - cur_yaw
        
        # Normalize diff to be between -π and π
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        if abs(diff) > tolerance:
            print(f'Yaw outside tolerance: {diff:.3f}')

            twist.angular.z = 1.0 * diff 

        else:
            # try to get back to initial
            twist.angular.z = 0.5 * diff 
    
            
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        twist = Twist()
        initial_dist = self.odom.dist  
        initial_yaw = self.odom.yaw 

        while not rospy.is_shutdown():
            dist_moved = self.odom.dist - initial_dist
            
            if dist_moved >= target_dist:  # Stop if the target distance is reached
                break
            self.maintain_yaw(initial_yaw, twist)
            # slow down when close to target
            remaining_dist = target_dist - dist_moved 
            if remaining_dist <= 0.25:
                if remaining_dist < 0.03:
                    twist.linear.x = 0.01
                elif remaining_dist < 0.1:
                    twist.linear.x = 0.05
                else:
                    twist.linear.x = 0.15
            else:
                # accelerate slower to avoid problems
                if dist_moved < 0.05:
                    twist.linear.x = 0.05
                elif dist_moved < 0.1:
                    twist.linear.x = 0.1
                else:
                    twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            
            self.rate.sleep()

        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def out_and_back(self, target_dist):
        """
        This function:
        1. Moves the robot forward by `target_dist`.
        2. Turns the robot by 180 degrees.
        3. Moves the robot forward by `target_dist`.
        Afterward, it calculates and prints the total error distance.
        """
        rospy.loginfo("Moving forward...")
        self.move_forward(target_dist) 
        rospy.loginfo("Turning 180 degrees...")
        self.turn_to_heading(math.pi)  
        rospy.loginfo("Moving back...")
        self.move_forward(target_dist)  
        rospy.loginfo("finished!")



    def get_error(self, initial_position, final_position):
        if initial_position and final_position:
            error_distance = math.sqrt(
                (final_position.x - initial_position.x) ** 2 +
                (final_position.y - initial_position.y) ** 2
            )
            rospy.loginfo(f'error: {error_distance:.4f}')


    def draw_square(self, side_length):
        current_yaw = self.odom.yaw

        for _ in range(4):
            self.move_forward(side_length)

            target_yaw = current_yaw + (math.pi / 2)

            # # Normalize the target yaw to be between -π and π
            # target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

            self.turn_to_heading(target_yaw)

            current_yaw = target_yaw

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        linear_velocity = 0.2 
        
        # Compute the angular velocity using ω = v / r
        angular_velocity = linear_velocity / r
        

        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('basic_mover')
    # BasicMover().out_and_back(1)
    # BasicMover().draw_square(1)
    # BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
    print('started')
    bot = BasicMover()
    # bot.move_forward(8)
    # Wait for the initial position to be available
    while bot.odom.old_pose is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for initial odometry data...")
        bot.rate.sleep()

    # step 1
    # initial = bot.odom.old_pose
    # bot.out_and_back(0.5)
    # final = bot.odom.old_pose
    # bot.get_error(initial, final)

    # setep 2
    initial = bot.odom.old_pose
    bot.draw_square(0.3)
    final = bot.odom.old_pose
    bot.get_error(initial, final)

    #step 3
    # bot.move_in_a_circle(0.3)