#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

def stop_robot(pub):
    msg = Twist()
    pub.publish(msg)
    rospy.sleep(1.0) 

def run_square():
    rospy.init_node('jetauto_square_node', anonymous=True)
    pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
    
    rospy.sleep(1.0)
    raw_input("Press Enter to start the 1-meter square sequence...")

    duration = 5.0 

    for loop in range(2):
        rospy.loginfo("--- Starting Loop %d ---", loop + 1)
        
        # STEP 1: Move forward
        rospy.loginfo("Step 1: Moving Forward")
        msg = Twist()
        msg.linear.x = 0.2
        pub.publish(msg)
        rospy.sleep(duration)
        stop_robot(pub)

        # STEP 2: Move sideways left
        rospy.loginfo("Step 2: Strafing Left")
        msg = Twist()
        msg.linear.y = 0.2
        pub.publish(msg)
        rospy.sleep(duration)
        stop_robot(pub)

        # STEP 3: Turn clockwise -90 degrees
        rospy.loginfo("Step 3: Turning Clockwise")
        msg = Twist()
        msg.angular.z = -0.314
        pub.publish(msg)
        rospy.sleep(duration)
        stop_robot(pub)

        # STEP 4: Move sideways right 
        rospy.loginfo("Step 4: Strafing Right")
        msg = Twist()
        msg.linear.y = -0.2
        pub.publish(msg)
        rospy.sleep(duration)
        stop_robot(pub)

        # STEP 5: The 100% Grade Dynamic Curve
        rospy.loginfo("Step 5: Curving back to Start (100% Grade Option)")
        
        # Run a control loop at 10 Hz to constantly update the wheel speeds
        rate = rospy.Rate(10) 
        t_start = rospy.Time.now().to_sec()
        
        while rospy.Time.now().to_sec() - t_start < duration:
            # Calculate how many seconds have passed in this step
            t = rospy.Time.now().to_sec() - t_start
            
            # Calculate current heading: Starts at -90 deg (-pi/2), ends at 0 deg
            theta = -math.pi/2 + (math.pi/2) * (t / duration)
            
            msg = Twist()
            # Dynamically shift from forward (linear.x) to strafing right (linear.y)
            msg.linear.x = -0.2 * math.sin(theta)
            msg.linear.y = -0.2 * math.cos(theta)
            msg.angular.z = 0.314 
            
            pub.publish(msg)
            rate.sleep()
            
        stop_robot(pub)
        
        rospy.loginfo("Loop %d complete.", loop + 1)

if __name__ == '__main__':
    try:
        run_square()
    except rospy.ROSInterruptException:
        pass
# Writing with the help of Gemini
