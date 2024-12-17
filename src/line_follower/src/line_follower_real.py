#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image

# video for sim and real: https://brandeis.zoom.us/rec/share/raUEU6OD2N48XAtJvdbOXPNqjD5YxFF3_YhyT0t8ZO-FFSH0DxFLSBiIs-rNu3IK.uwv0X67SY4Lq28E3?startTime=1731023783000
# video for code explanation: https://brandeis.zoom.us/rec/share/d2RkqmLuyr-EKaiSva_FYq4a41jfDsO3fDAJu5L9w67YgGChnrHEx_BU-GXhCIZs.9q_-rRqy5qTxyDf8?startTime=1731024222000

class LineFollowerReal:
    def __init__(self):
        # cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        # self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
        #                                   Image,
        #                                   self.image_cb)
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.follow_line = False
        self.desired_distance = 0.3
        self.twist = Twist()
        self.count = 0
        self.image = None
        # self.masked_image = None
        self.scan_data = None
        self.set_up_count = 0

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

    # def my_odom_cb(self, msg):
    #     """Callback to `self.my_odom_sub`."""
    #     raise NotImplementedError

    def image_cb(self, msg):
        # self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np_arr = numpy.frombuffer(msg.data, numpy.uint8)
        self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    def process_image(self):
        image = self.image

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define the lower and upper limits for yellow color
        # lower_yellow = numpy.array([10, 10, 10])
        # upper_yellow = numpy.array([255, 255, 250])
        lower_red = numpy.array([170,100,100])
        upper_red = numpy.array([180, 255,255])
        
        # Create a mask that isolates the yellow line in the image
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        h, w, d = image.shape
        search_top = 3 * h // 4
        search_bot = search_top + 20
        
        # Mask out everything except the region of interest
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        # Calculate the moments of the mask to find the center of the line
        M = cv2.moments(mask)

        cardinal_directions = self.cardinal_directions()
        
        if M['m00'] > 0 and cardinal_directions['Front'] > self.desired_distance:
            self.set_up_count = 0
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Draw a circle at the centroid of the detected line
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            
            # Compute the error (difference from the center of the image)
            err = (cx - w // 2) / 350
            
            # Cap the angular velocity to avoid crazy values
            max_angular_speed = 1.0  
            angular_z = -float(err) # Scale down the error to control sensitivity
            angular_z = max(min(angular_z, max_angular_speed), -max_angular_speed)
            
            # Set linear and angular velocities to control the robot
            self.twist.angular.z = angular_z
            self.twist.linear.x = 0.1
        else:
            print('M is not greater than 0')
            if self.set_up_count < 17:
                # print(cardinal_directions['Front'])
                print('count is ', self.set_up_count)
                self.twist.angular.z = 0.5
                self.twist.linear.x = 0
                self.set_up_count += 1
            
            else:
                if self.scan_data:
                    print('front: ', cardinal_directions['Front'], 'right: ', cardinal_directions['Right'])
                    # Process scan data and implement wall-following behavior here
                    min_distance, min_angle = self.find_min_range_and_angle(self.scan_data)
                    normalized_angle = (min_angle + math.pi) % (2 * math.pi) - math.pi
                    # print('cardinal_directions front:', cardinal_directions['Front'], 'right', cardinal_directions['Right'])
                    if cardinal_directions["Front"] > 1.2 * self.desired_distance:
                        if cardinal_directions["Right"] < 1.2 * self.desired_distance:
                            self.trace_wall(cardinal_directions["Right"], normalized_angle, self.twist)
                        else:
                            # # turn right
                            # if cardinal_directions["Front"] < 0.8 * self.desired_distance and cardinal_directions["Right"] < float('inf'):
                            #     self.twist.linear.x = 0
                            #     self.twist.angular.z = 0.5
                            print("turn right")
                            self.twist.linear.x = 0.1
                            self.twist.angular.z = -0.8

                    else:
                        # need to turn left
                        print("turn left")
                        self.twist.linear.x = 0.1
                        self.twist.angular.z = 0.8


            

        
        self.cmd_vel_pub.publish(self.twist)
    
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
            result = self.averager(data)
            if result == 0 and direction == 'Right':
                result = 5
            directions[direction] = result
        
        return directions

    def preprocessor(self, all_ranges):
        ranges = [float('inf')] * 360  # Initialize processed ranges array for 360 values
        min_detectable_distance = 5 # Minimum detectable distance

        for i in range(360):
            index = (i - 9) % len(all_ranges)  # Ensure the index wraps around
            curr = all_ranges[index]
            
            # If the current reading is too close (inf or NaN), cap it at the minimum distance
            if curr == float('inf') or math.isnan(curr):
                curr = min_detectable_distance  # Cap at the minimum distance
            
            # Filter out invalid and too-close values

            ranges[i] = curr  # Directly assign the valid reading

        return ranges
        
    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        self.scan_data = self.preprocessor(msg.ranges)

    def trace_wall(self, min_distance, min_angle, twist):
        """
        Simplified PID-based wall-following behavior.
        Adjusts angular velocity based on both the distance error and angular error.
        """
        if not self.scan_data:
            return
        
        # error = min_distance - self.desired_distance
        error = min_distance - self.desired_distance
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
        twist.angular.z = -distance_control
        #  + (angular_control)
        twist.linear.x = 0.1  # Move forward at a constant speed
          
    # def follow_line(self):
    #     """Follow a black line."""
    #     raise NotImplementedError

    # def avoid_obstacle(self):
    #     """Avoid an obstacle of known dimensions."""
    #     raise NotImplementedError
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(7)
        

        while not rospy.is_shutdown():
            if self.image is not None and self.image.any():
                self.process_image()

            # if self.follow_line:
            #     self.follow_line()
            # else:
            #     self.avoid_obstacle()
            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
