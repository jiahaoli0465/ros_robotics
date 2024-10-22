import math

import math

def find_min_range_and_angle(ranges):
    min_average_range = float('inf')
    best_angle_start = None
    
    # Loop through all 360 range values (assuming 1 degree increments)
    for angle in range(len(ranges) - 9):  # We stop at len(ranges) - 9 to avoid index out of range
        # Get the next 10 consecutive ranges
        consecutive_ranges = ranges[angle:angle + 10]
        
        # Filter out invalid data (infinity and NaN)
        valid_ranges = [r for r in consecutive_ranges if r != float('inf') and not math.isnan(r)]
        
        # Skip if all data in the window is invalid
        if not valid_ranges:
            continue
        
        # Calculate the average of the valid ranges
        average_range = sum(valid_ranges) / len(valid_ranges)
        
        # Find the smallest average range
        if average_range < min_average_range:
            min_average_range = average_range
            best_angle_start = angle  # Start of the 10-degree window

    # Return the average angle in the best 10-degree window
    if best_angle_start is not None:
        avg_angle = best_angle_start + 4.5  # Middle of the 10-degree window
        return min_average_range, math.radians(avg_angle)
    
    return None, None  # In case no valid data is found


def averager(direction_ranges):
    # source from https://campusrover.github.io/labnotebook2/faq/Lidar/Simplifying_Lidar/
    real = 0
    sum = 0
    for i in direction_ranges:
        if  0.2 < i < 2 and not math.isnan(i):
            real += 1
            sum += i  
    return float('inf') if real == 0 else sum / real