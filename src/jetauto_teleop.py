#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions to display in the terminal
msg = """
Control Your JetAuto in Gazebo!
---------------------------
Moving around:
   w    
a  s  d

q/e : turn CCW/CW
spacebar : force stop
CTRL-C to quit
"""

def getKey():
    # This function captures single key presses without needing to hit 'Enter'
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    # 1. Initialize the ROS Node
    rospy.init_node('lab4_teleop_node')
    
    # 2. Create the Publisher targeting the JetAuto Gazebo topic
    pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
    
    # Safe speeds for Gazebo simulation (Max linear is 0.7, Max angular is 1.0)
    speed = 0.3
    turn = 0.5
    
    x = 0
    y = 0
    th = 0
    
    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            
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
            elif key == ' ':
                x, y, th = 0, 0, 0    # Stop
            else:
                if (key == '\x03'):   # Catch CTRL-C to exit cleanly
                    break

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
