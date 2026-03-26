#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LidarEvader:
    def __init__(self):
        rospy.init_node('lidar_evader_node', anonymous=True)
        self.pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
        
        self.path_clear = True
        self.closest_distance = 0.0
        self.rate = rospy.Rate(10)

    def lidar_callback(self, msg):
        total_rays = len(msg.ranges)
        
        # 1. Define the "Donut" boundaries
        BODY_RADIUS = 0.4  # Ignore anything closer than 40cm (The robot's own arm/screen)
        WARNING_ZONE = 1.0 # React to anything between 40cm and 1 meter (The boxes)
        
        # 2. Slice a wider 90-degree headlight cone to see the edges of the boxes
        cone_width = total_rays // 4
        half_cone = cone_width // 2
        front_ranges = msg.ranges[:half_cone] + msg.ranges[-half_cone:]
        
        # 3. Apply the Donut Filter
        valid_ranges = [r for r in front_ranges if BODY_RADIUS < r < WARNING_ZONE and r != float('inf')]
        
        # 4. State Management
        if len(valid_ranges) > 0:
            self.path_clear = False
            self.closest_distance = min(valid_ranges)
        else:
            self.path_clear = True

    def run(self):
        rospy.loginfo("LiDAR Evader Active: Exploring...")
        
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.path_clear:
                # Coast is clear, drive straight forward
                twist.linear.x = 0.3
                twist.angular.z = 0.0
            else:
                # Obstacle detected! Print the exact distance to the terminal for debugging
                rospy.loginfo("OBSTACLE at %.2f meters! Turning...", self.closest_distance)
                
                # Stop forward momentum and turn
                twist.linear.x = 0.0
                twist.angular.z = 0.8 
                
            self.pub.publish(twist)
            self.rate.sleep()
            
        self.pub.publish(Twist())

if __name__ == '__main__':
    try:
        evader = LidarEvader()
        evader.run()
    except rospy.ROSInterruptException:
        pass
