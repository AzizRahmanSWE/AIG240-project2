#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions to display in the terminal
msg = """
Control Your JetAuto in Gazebo!
---------------------------
Moving around (HOLD DOWN KEY):
   w    
a  s  d

q/e : turn CCW/CW
Release key to automatically stop!
CTRL-C to quit
"""

def getKey(timeout=0.1):
    # Added a timeout so it doesn't block forever waiting for a key
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''  # Return empty string if no key is pressed
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    # 1. Initialize the ROS Node
    rospy.init_node('lab4_teleop_node')
    
    # 2. Create the Publisher targeting the JetAuto Gazebo topic
    pub = rospy.Publisher('/jetauto_1/cmd_vel', Twist, queue_size=10)
    
    # SLOWED DOWN: Reduced speed for careful mapping
    speed = 0.1  # Down from 0.3
    turn = 0.2   # Down from 0.5
    
    x = 0
    y = 0
    th = 0
    
    try:
        print(msg)
        while not rospy.is_shutdown():
            # Check for a key press every 0.1 seconds
            key = getKey(0.1)
            
            # Map keyboard inputs to movement vectors
            if key == 'w':
                x, y, th = 1, 0, 0    # Forward
            elif key == 's':
                x, y, th = -1, 0, 0   # Backward
            elif key == 'a':
                x, y, th = 0, 1, 0    # Strafe Left
            elif key == 'd':
                x, y, th = 0, -1, 0   # Strafe Right
            elif key == 'q':
                x, y, th = 0, 0, 1    # Turn Counter-Clockwise
            elif key == 'e':
                x, y, th = 0, 0, -1   # Turn Clockwise
            elif key == '\x03':       # Catch CTRL-C to exit cleanly
                break
            else:
                # AUTO-STOP: If you let go of the keys, zero out the speeds
                x, y, th = 0, 0, 0    

            # 3. Construct and Publish the Twist message
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)
        
    finally:
        # 4. Always send a zero-velocity stop command before shutting down
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
