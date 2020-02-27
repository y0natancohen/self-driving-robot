#!/usr/bin/env python
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import sys
from dijkstar import Graph, find_path
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import cv2
import time

from graph2 import angles, nav_types, points, generate_graph, line, amcl, distance

pi = math.pi

twist = Twist()
cmd_vel_pub = None
bridge = None
inf_time = rospy.Duration(99999)
move_base = None
positive_count = 0
negative_count = 0

goal_failed = False
min_distance_to_goal = 999
curr_goal = None
stopping = False
do_line_detection = False


def make_goal(p, angle):
    if isinstance(p, tuple):
        x, y = p
    else:
        x, y = points[p]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    p = Point(x, y, 0)
    q = quaternion_from_euler(0, 0, angle)
    q = Quaternion(*q)
    goal.target_pose.pose = Pose(p, q)
    return goal


def init_move_base():
    global move_base
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(timeout=inf_time)  # wait to infinity


def navigate_to_goal_amcl(p, angle):
    global goal_failed, min_distance_to_goal
    min_distance_to_goal = 999
    goal = make_goal(p, angle)
    move_base.send_goal(goal, feedback_cb=_callback_feedback)
    print "amcl: going to goal {}, line_detection={}".format(p, do_line_detection)
    result = move_base.wait_for_result(timeout=inf_time)  # wait to infinity
    state = move_base.get_state()
    if (result and state) == GoalStatus.SUCCEEDED:
        print("amcl: arrived!")
        return True
    else:
        print("amcl: didnt arrive,  {}".format(result and state))
        return False


def stop():
    global stopping
    stopping = True
    # 1.5 secs
    for i in range(150):
        twist.angular.z = 0
        twist.linear.x = 0
        cmd_vel_pub.publish(twist)
        time.sleep(0.01)
    stopping = False


def _callback_feedback(feedback):
    global min_distance_to_goal
    curr_pose = feedback.base_position.pose.position
    x, y = points[curr_goal]
    dist = distance((curr_pose.x, curr_pose.y), (x, y))
    if dist < min_distance_to_goal:
        min_distance_to_goal = dist

    if dist > min_distance_to_goal + 0.8:
        print "wrong way!! canceling goal!!"
        move_base.cancel_goal()


def get_goals_order():
    """ prgb translates to (pink -> red -> green -> blue) """
    if len(sys.argv) < 2:
        raise Exception('usage: navigate.py "pgrb" (the order of the colors) or navigate.py goto n1')
    points_names = {'r': 'red', 'p': 'n4_stop_line', 'g': 'c3', 'b': 'blue'}

    print(sys.argv[1])
    print([char for char in sys.argv[1]])
    return [points_names[char] for char in sys.argv[1]]


def navigate(src, dest, g):
    global curr_goal, do_line_detection
    path = find_path(g, s=src, d=dest)
    print "planned path from {} to {} is {}".format(src, dest, path.nodes)
    edges = [(path.nodes[i], path.nodes[i+1]) for i in range(len(path.nodes[:-1]))]
    for i, edge in enumerate(edges):
        curr_goal = edge[1]
        angle = angles[edge]
        nav_type = nav_types[edge]
        if nav_type == line:
            do_line_detection = True
        else:
            do_line_detection = False

        success = navigate_to_goal_amcl(edge[1], angle)
        if not success:
            print "first try wasn't good :( try again:"
            success2 = navigate_to_goal_amcl(edge[1], angle)
            if not success2:
                print "second try wasn't good :( try again:"
                success3 = navigate_to_goal_amcl(edge[1], angle)
                if not success3:
                    print "third try wasn't good :( try again:"
                    success4 = navigate_to_goal_amcl(edge[1], angle)
                    if not success4:
                        print "fourth try wasn't good"
                        print "all is lost :("

        if edge[1].endswith('stop_line') or edge[1] == 'e4':
            print "stopping at a stop line"
            stop()
            time.sleep(1.7)

    print "finished path from {} to {}".format(src, dest)


def camera_callback(image_data):
    if stopping:
        return
    if not do_line_detection:
        return
    global positive_count, negative_count
    image = bridge.imgmsg_to_cv2(image_data, 'bgr8')
    h, w, d = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 10, 10])
    upper_yellow = np.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    search_top = 3*h/4
    search_bot = 3*h/4 + 50
    mask[:search_top, :] = 0
    mask[search_bot:, :] = 0
    mask[:, 0:w/3] = 0  # only look at the line to the right
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return

    cx = int(M['m10']/M['m00'])
    desired = (w/2) + 550
    err = cx - desired
    rotation = -(err / 600.0)
    max_theta_speed = 0.6  # left
    min_theta_speed = -0.9  # right

    if rotation > max_theta_speed:  # n right
        rotation = max_theta_speed
    if rotation < min_theta_speed:  # turn left
        rotation = min_theta_speed

    speed = 0.3
    twist.angular.z = rotation
    twist.linear.x = speed
    cmd_vel_pub.publish(twist)


def navigator():
    global cmd_vel_pub, bridge
    bridge = CvBridge()
    rospy.init_node('navigator', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback)
    # rospy.Subscriber("/scan", LaserScan, scanner_callback)
    # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    g = generate_graph()
    init_move_base()

    goals = get_goals_order()

    navigate('start', goals[0], g)
    navigate(goals[0], goals[1], g)
    navigate(goals[1], goals[2], g)
    navigate(goals[2], goals[3], g)
    print "that's all for today"


if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
