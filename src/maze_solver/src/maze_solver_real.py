#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from logger import Logger

# video: https://brandeis.zoom.us/rec/share/84diJ_iXuypQLkEQewliuUIA07g0w42QTeHIwIh8F7AfMqy-gnMo0kkjkN77i_UZ.FGgyBnmpKhjiDG7V


class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.scan_data = None
        self.set_up = False
        self.rate = rospy.Rate(10)
        self.desired_distance = 0.35
        self.error_range = 0.05


    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        self.scan_data = self.preprocessor(msg.ranges)
    def preprocessor(self, all_ranges):
        ranges = [float('inf')] * 360  # Initialize processed ranges array for 360 values
        min_detectable_distance = 0.12  # Minimum detectable distance

        for i in range(360):
            index = (i - 9) % len(all_ranges)  # Ensure the index wraps around
            curr = all_ranges[index]
            
            # If the current reading is too close (inf or NaN), cap it at the minimum distance
            if curr == float('inf') or math.isnan(curr):
                curr = min_detectable_distance  # Cap at the minimum distance
            
            # Filter out invalid and too-close values
            if curr >= min_detectable_distance:
                ranges[i] = curr  # Directly assign the valid reading

        return ranges




    def find_min_range_and_angle(self, ranges):
        min_range = float('inf')
        min_angle = None
        
        # Loop through all 360 range values (assuming 1 degree increments)
        for angle in range(len(ranges)):
            curr_range = ranges[angle]
            if curr_range != float('inf') and not math.isnan(curr_range):  # Ignore invalid data
                if curr_range < min_range:
                    min_range = curr_range
                    min_angle = angle  # The angle is the index in the ranges array

        return min_range, math.radians(min_angle)

    def averager(self, direction_ranges):
        # source from https://campusrover.github.io/labnotebook2/faq/Lidar/Simplifying_Lidar/
        real = 0
        sum = 0
        for i in direction_ranges:
            if i != float('inf') and not math.isnan(i):
                real += 1
                sum += i  
        return float('inf') if real == 0 else sum / real

    def cardinal_directions(self):
        ranges = self.scan_data
        directions = {"Front": float('inf'), "Back": float('inf'), "Left": float('inf'), "Right": float('inf')}
        plus_minus = 9
        Front = ranges[-plus_minus:] + ranges[:plus_minus]
        Back = ranges[180 - plus_minus:180 + plus_minus]
        Left = ranges[90 - plus_minus:90 + plus_minus]
        Right = ranges[270 - plus_minus:270 + plus_minus]
        
        directions_before_processed = {"Front": Front, "Back": Back, "Left": Left, "Right": Right}
        

        
        for direction, data in directions_before_processed.items():
            directions[direction] = self.averager(data)
        
        return directions


    def init_to_wall(self, min_distance, normalized_angle, twist):
        if min_distance > self.desired_distance:
            # lets get to the wall before going to trace it.
            if -0.12 < normalized_angle < 0.12:
                twist.linear.x = 0.15
                
            twist.angular.z = 0.3 * normalized_angle
        else:
            # lets make the robot parallel
            desired_angle = -(math.pi / 2  )
            if desired_angle - 0.05 < normalized_angle < desired_angle + 0.05:
                twist.angular.z = 0
                self.set_up = True
                print('set up completed')
            else:
                twist.angular.z = 0.2
    
    def trace_wall(self, min_distance, min_angle, twist):
        """
        Simplified PID-based wall-following behavior.
        Adjusts angular velocity based on both the distance error and angular error.
        """
        if not self.scan_data:
            return
        
        # error = min_distance - self.desired_distance
        error = min_distance - 0.3
        # Compute the angular error from parallel (for right wall-following, target is -pi/2)
        desired_angle = -math.pi / 2  # wall is on the right side for this
        error_angular = min_angle - desired_angle

        # Cap the angular error if necessary to prevent over-adjustment
        if error_angular < -math.pi:
            error_angular += 2 * math.pi
        elif error_angular > math.pi:
            error_angular -= 2 * math.pi

        # Combine the distance error and angular error into the control
        distance_control = 0.5 * error  # Proportional control for distance
        angular_control = 0.5 * error_angular  # Proportional control for angle

        # Limit the control values to avoid excessive turning
        if distance_control < -0.7:
            distance_control = -0.7
        elif distance_control > 0.7:
            distance_control = 0.7

        if angular_control < -0.7:
            angular_control = -0.7
        elif angular_control > 0.7:
            angular_control = 0.7

        # Adjust the robot's angular velocity to correct both distance and angle
        twist.angular.z = -distance_control + (angular_control)
        twist.linear.x = 0.1  # Move forward at a constant speed

        # Log errors for debugging
        print(f"Distance Error: {error}, Angular Error: {error_angular}, Control: {twist.angular.z}")


    def follow_wall(self):
        dis_logger = Logger()

        while not rospy.is_shutdown():
            if self.scan_data:
                # Process scan data and implement wall-following behavior here
                min_distance, min_angle = self.find_min_range_and_angle(self.scan_data)
                normalized_angle = (min_angle + math.pi) % (2 * math.pi) - math.pi
                


 
                dis_logger.log(f"dist: {min_distance}, ang: {normalized_angle}")

                twist = Twist()

                cardinal_directions = self.cardinal_directions()

                
                if cardinal_directions["Front"] > 1.2 * self.desired_distance:
                    if cardinal_directions["Right"] < 1.2 * self.desired_distance:
                        self.trace_wall(cardinal_directions["Right"], normalized_angle, twist)
                    else:
                        # turn right
                        if cardinal_directions["Front"] < 0.8 * self.desired_distance and cardinal_directions["Right"] < float('inf'):
                            twist.linear.x = 0
                            twist.angular.z = 0.5
                        print("turn right")
                        twist.linear.x = 0.2
                        twist.angular.z = -0.8

                else:
                    # need to turn left
                    print("turn left")
                    twist.linear.x = 0
                    twist.angular.z = 0.8

                print('front_distance: ', cardinal_directions["Front"])
                print('right d: ', cardinal_directions["Right"])
                self.cmd_vel_pub.publish(twist)
            

            self.rate.sleep()
    

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()



# class exercise
# range = data.ranges[350:] + data.ranges[:10]
# range = [r in range if not math.isnan(r) and  0.12 < r < 3.5]
# avg = sum(ranges) / len(ranges)
# return avg < 0.3