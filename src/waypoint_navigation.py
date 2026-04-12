#!/usr/bin/env python
import math
import rospy
import numpy as np
import tf  # Replaced jetauto_sdk with standard ROS tf library
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# Define your array of waypoints here: (x, y, orientation_in_degrees)
waypoints = [
    (-0.825, 0.2, 0.0),
    (2.50, 1.6, 0.0),
    (0.88, -0.5, 0.0),
    (0.27, 2.26, 0.0),
]

current_wp_index = 0
markerArray = MarkerArray()

def set_current_waypoint():
    global current_wp_index, markerArray

    if current_wp_index >= len(waypoints):
        rospy.loginfo("=====================================================")
        rospy.loginfo("STATUS UPDATE: All waypoints reached!")
        rospy.loginfo("=====================================================")
        return

    target_x, target_y, target_yaw = waypoints[current_wp_index]
    rospy.loginfo("STATUS UPDATE: Setting goal for Waypoint {}/{} -> x: {}, y: {}, yaw: {}".format(current_wp_index + 1, len(waypoints), target_x, target_y, target_yaw))

    # Clear previous markers
    marker_Array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = map_frame
    marker.action = Marker.DELETEALL
    marker_Array.markers.append(marker)
    mark_pub.publish(marker_Array)

    # Create Pose
    pose = PoseStamped()
    pose.header.frame_id = map_frame
    pose.header.stamp = rospy.Time.now()

    # NATIVE ROS QUATERNION CONVERSION (Bypassing smbus2 error)
    q_tf = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(target_yaw))
    pose.pose.position.x = target_x
    pose.pose.position.y = target_y
    pose.pose.orientation.x = q_tf[0]
    pose.pose.orientation.y = q_tf[1]
    pose.pose.orientation.z = q_tf[2]
    pose.pose.orientation.w = q_tf[3]

    # Set up the visual marker (flag) in RViz
    marker = Marker()
    marker.header.frame_id = map_frame
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://rviz_plugin/media/flag.dae"
    marker.action = marker.ADD
    marker.scale.x = 0.08
    marker.scale.y = 0.08
    marker.scale.z = 0.2

    # Randomize marker color
    color = list(np.random.choice(range(256), size=3))
    marker.color.a = 1
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0

    marker.pose.position.x = pose.pose.position.x
    marker.pose.position.y = pose.pose.position.y
    marker.pose.orientation = pose.pose.orientation

    markerArray.markers.clear()
    markerArray.markers.append(marker)

    # Publish marker and navigation goal
    mark_pub.publish(markerArray)
    goal_pub.publish(pose)
    rospy.loginfo("STATUS UPDATE: Robot is now moving to Waypoint {}...".format(current_wp_index + 1))

def status_callback(msg):
    global current_wp_index

    status = msg.status.status

    # Status 3 means the goal was reached successfully
    if status == 3:
        rospy.loginfo('STATUS UPDATE: Waypoint {} reached successfully.'.format(current_wp_index + 1))
        current_wp_index += 1

        if current_wp_index < len(waypoints):
            rospy.loginfo("STATUS UPDATE: Preparing for the next waypoint in 2 seconds...")
            rospy.sleep(2.0)
            set_current_waypoint()
        else:
            set_current_waypoint() # This will trigger the "Navigation complete" block

    # Handle failure states
    elif status == 4:
        rospy.logwarn('STATUS UPDATE: Waypoint {} was aborted by move_base. Stopping sequence.'.format(current_wp_index + 1))
    elif status == 5:
        rospy.logwarn('STATUS UPDATE: Waypoint {} was rejected by move_base. Stopping sequence.'.format(current_wp_index + 1))
    elif status == 9:
        rospy.logwarn('STATUS UPDATE: Waypoint {} was lost. Stopping sequence.'.format(current_wp_index + 1))

if __name__ == '__main__':
    rospy.init_node('key_start_waypoint_nav', anonymous=True)

    map_frame = rospy.get_param('~map_frame', 'jetauto_1/map')
    move_base_result = rospy.get_param('~move_base_result', '/jetauto_1/move_base/result')

    goal_pub = rospy.Publisher('/jetauto_1/move_base_simple/goal', PoseStamped, queue_size=1)
    mark_pub = rospy.Publisher('jetauto_1/path_point', MarkerArray, queue_size=100)

    rospy.Subscriber(move_base_result, MoveBaseActionResult, status_callback)

    # Give a brief moment for publishers to register with the ROS Master
    rospy.sleep(0.5)

    rospy.loginfo("STATUS UPDATE: Node initialized and ready.")

    # Wait for the user to press Enter in the terminal
    try:
        raw_input(">>> Press [ENTER] to begin the navigation sequence... <<<")
    except NameError:
        input(">>> Press [ENTER] to begin the navigation sequence... <<<")

    rospy.loginfo("STATUS UPDATE: Starting waypoint sequence!")
    set_current_waypoint()

    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
