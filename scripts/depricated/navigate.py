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
import datetime
import threading, thread
from apscheduler.schedulers.background import BackgroundScheduler
import time

from graph import angles, nav_types, points, generate_graph, line, amcl, distance
from obstacle import is_obstacle

scheduler = BackgroundScheduler()

pi = math.pi

twist = Twist()
cmd_vel_pub = None
bridge = None
obstacle_found = False
inf_time = rospy.Duration(99999)
move_base = None
position = (3.4, -1.6)  # initial
orientation = 0
do_line_navigation = True
# line_goal_reached = False
# goal_accuracy_delta = 0.25
goal_accuracy_delta = 0.3
theta_accuracy_delta = 0.2
obstacle_threshold = 0.8
turn_left_angular_speed = 1.3
go_around_delta = 1.6
positive_count = 0
negative_count = 0
rotate_right_slowly_speed = -0.5
goal_failed = False
lost_the_line = False
lost_the_line_time = None
retry_obstacle = False
retry_obstacle_count = 0
amcl_goal_canceled_because_of_obstacle = False

# TODO: fill points coordinates
# TODO: how to use the scan laser to avoid obstacles? (during the way to the goal) maybe changing package configuration
# TODO: stay inside the lines - integrate code from ass2
# TODO: need to stop at a white line before a turn
# TODO: after everything, edit the map and remove current example obstacles


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
    # move_base.wait_for_server()
    move_base.wait_for_server(timeout=inf_time)  # wait to infinity


def navigate_to_goal_amcl(p, angle, in_order_to_obstacle=False):
    global goal_failed
    goal = make_goal(p, angle)
    # callback only for regular goals
    if in_order_to_obstacle:
        move_base.send_goal(goal)
    else:
        move_base.send_goal(goal, feedback_cb=_callback_feedback)
    # move_base.send_goal(goal)
    print "amcl: going to goal {}, angle at the end: {} ...".format(p, angle)
    result = move_base.wait_for_result(timeout=inf_time)  # wait to infinity
    state = move_base.get_state()
    if (result and state) == GoalStatus.SUCCEEDED:
        print("amcl: arrived!")
    else:
        if not in_order_to_obstacle and obstacle_found:
            print("amcl: didnt arrive because going around an obstacle")
            avoid_obstacle()
            # wait for obstacle clear
            # while obstacle_found:
            #     time.sleep(0.1)
            print("amcl: obstacle avoided, should be clear")
        else:
            goal_failed = True
            print("amcl: didnt arrive,  {}".format(result and state))


def stop():
    # print "stopping"
    for i in range(10):
        twist.angular.z = 0
        twist.linear.x = 0
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)


def turn_right():
    print "turning right"
    # initial_orientation = orientation
    twist.angular.z = -turn_left_angular_speed
    cmd_vel_pub.publish(twist)
    time.sleep(1.1)
    stop()


def rotate_right_slowly():
    print "rotate right slowly to find the line..."
    # initial_orientation = orientation
    twist.angular.z = rotate_right_slowly_speed
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)


def turn_left():
    print "turning left"
    # initial_orientation = orientation
    twist.angular.z = turn_left_angular_speed
    cmd_vel_pub.publish(twist)
    time.sleep(1.5)
    stop()

    # wanted = initial_orientation + pi/2
    # while True:
    #     curr_orientation = orientation
    #     twist.angular.z = turn_left_angular_speed
    #     cmd_vel_pub.publish(twist)
    #     print curr_orientation, wanted
    #     if abs(curr_orientation - wanted) < theta_accuracy_delta:
    #         break
    #     time.sleep(0.01)
    # twist.angular.z = 0
    # cmd_vel_pub.publish(twist)
    # # curr_position = position
    # navigate_to_goal_amcl(curr_position, curr_orientation + pi/2)


def go_around(initial_position, initial_orientation):
    x, y = initial_position
    delta = go_around_delta

    # calculate next orientation
    axes = np.array([0, pi/2, pi, 3*pi/2], dtype=np.float32)
    diffs = np.abs(axes - initial_orientation)
    closest_ax_index = np.where(diffs == np.amin(diffs))[0][0]
    closest_ax = axes[closest_ax_index]
    print "angle: {}, closest: {}".format(initial_orientation, closest_ax)

    new_pos = (x + (math.cos(closest_ax)*delta),
               y + (math.sin(closest_ax))*delta)
    navigate_to_goal_amcl(new_pos, closest_ax, in_order_to_obstacle=True)


def step_forward():
    print "stepping forward"
    twist.linear.x = 0.2
    cmd_vel_pub.publish(twist)
    time.sleep(2.6)
    stop()


def step_back(little=False):
    print "stepping back"
    twist.linear.x = -0.2
    cmd_vel_pub.publish(twist)
    if little:
        time.sleep(1.7)
    else:
        time.sleep(2.7)
    stop()

    # initial_orientation = orientation
    # initial_position = position
    #
    # delta = 0.6
    # x, y = initial_position
    # new_pos = (x + (math.cos(initial_orientation)*delta),
    #            y + (math.sin(initial_orientation))*delta)

    # while True:
    #     twist.linear.x = 0.2
    #     cmd_vel_pub.publish(twist)
    #     if abs(distance((x, y), new_pos)) < goal_accuracy_delta:
    #         break
    #     time.sleep(0.01)
    # twist.linear.x = 0
    # cmd_vel_pub.publish(twist)


def avoid_obstacle():
    curr_orientation = orientation
    curr_position = position
    stop()
    step_back()
    time.sleep(1)
    turn_left()
    step_forward()
    turn_right()
    go_around(curr_position, curr_orientation)


def _line_gaol_checkup(x, y, angle):
    global obstacle_found
    while True:
        # print position, (x, y), goal_accuracy_delta
        # print('d: {}'.format(distance(position, (x, y))))
        if goal_failed:
            break
        if distance(position, (x, y)) < goal_accuracy_delta:
            break
        if abs(position[0] - x) < (goal_accuracy_delta/2) and abs(position[1] - y) < (goal_accuracy_delta*1.2):
            break
        if abs(position[1] - y) < (goal_accuracy_delta/2) and abs(position[0] - x) < (goal_accuracy_delta*1.2):
            break
        if retry_obstacle:
            time.sleep(0.1)
            continue
        if obstacle_found:
            print "obstacle_found"
            # if not do_line_navigation:  # amcl
                # wait for the previous goal to cancel
                # while True:
                #     if amcl_goal_canceled_because_of_obstacle:
                #         break
            # go around
            # print "obstacle_found stop"

            avoid_obstacle()
            # curr_orientation = orientation
            # curr_position = position
            # stop()
            # time.sleep(1)
            # # print "obstacle_found stop"
            # turn_left()
            # step_forward()
            # turn_right()
            # go_around(curr_position, curr_orientation)
            # obstacle_found = False

        time.sleep(0.1)


class MyThread(threading.Thread):
    def __init__(self, x, y, angle):
        threading.Thread.__init__(self)
        self.x = x
        self.y = y
        self.angle = angle

    def run(self):
        print "strating driver thread"
        _line_gaol_checkup(self.x, self.y, self.angle)
        # print "finishing thread"


def navigate_to_goal_line(p, angle):
    global do_line_navigation
    do_line_navigation = True
    x, y = points[p]
    # scheduler.add_job(_line_gaol_checkup, )
    # line_goal_reached = False
    print "line: going to goal {}, angle at the end: {} ...".format(p, np.round(angle, 2))
    t1 = MyThread(x, y, angle)
    t1.daemon = True
    t1.start()
    while True:
        t1.join(500)
        if not t1.isAlive():
            break
    # print "t1 finished"
    # thread.start_new_thread(_line_gaol_checkup, (x, y))
    # thread.start_new_thread(_line_gaol_checkup, (x, y))
    # while True:
    #     # print position, (x, y), goal_accuracy_delta
    #     print('d: {}'.format(distance(position, (x, y))))
    #     if distance(position, (x, y)) < goal_accuracy_delta:
    #         break
    #     if obstacle_found:
    #         curr_orientation = orientation
    #         curr_position = position
    #         stop()
    #         turn_left()
    #         go_around(curr_position, curr_orientation)
    #     time.sleep(0.1)

    stop()
    do_line_navigation = False
    if goal_failed:
        print("line: failed to arrive :(")
    else:
        print("line: arrived!")
    # line_goal_reached = True


# def _callback_done(*args):
#     print ("_callback_done")
#     print ("*args: {}".format(args))
#
#
# def _callback_active(*args):
#     print ("_callback_active")
#     print ("*args: {}".format(args))
#
#
# do = True
#
#
def _callback_feedback(feedback):
    if not isinstance(move_base, actionlib.SimpleActionClient):
        raise
    # global do
    # if do:
    #     print feedback
    #     do = False
    # global obstacle_found
    # curr_pose = feedback.base_position.pose
    # if not isinstance(curr_pose, Pose):
    #     raise
    global amcl_goal_canceled_because_of_obstacle
    if obstacle_found:
        move_base.cancel_goal()
        # move_base.cancel_all_goals()
        print("amcl goal canceled, obstacle found")
        amcl_goal_canceled_because_of_obstacle = True


def get_goals_order():
    """ prgb translates to (pink -> red -> green -> blue) """
    if len(sys.argv) < 2:
        raise Exception('usage: navigate.py "pgrb" (the order of the colors) or navigate.py goto n1')
    points_names = {'r': 'red', 'p': 'n4_stop_line', 'g': 'c3', 'b': 'blue'}

    print(sys.argv[1])
    print([char for char in sys.argv[1]])
    return [points_names[char] for char in sys.argv[1]]


def navigate(src, dest, g):
    global do_line_navigation, goal_failed, retry_obstacle_count, amcl_goal_canceled_because_of_obstacle
    path = find_path(g, s=src, d=dest)
    print "planned path from {} to {} is {}".format(src, dest, path.nodes)
    edges = [(path.nodes[i], path.nodes[i+1]) for i in range(len(path.nodes[:-1]))]
    for edge in edges:
        goal_failed = False
        angle = angles[edge]
        nav_type = nav_types[edge]
        if nav_type == amcl:
            do_line_navigation = False
            amcl_goal_canceled_because_of_obstacle = False
            navigate_to_goal_amcl(edge[1], angle)
            if goal_failed:
                step_back()
                navigate_to_goal_amcl(edge[1], angle)
        else:
            do_line_navigation = True
            retry_obstacle_count = 0
            navigate_to_goal_line(edge[1], angle)
            if goal_failed:
                navigate_to_goal_amcl(edge[1], angle)

        if edge[1].endswith('stop_line'):
            print "stopping at a stop line"
            stop()
            time.sleep(1)
    print "finished path from {} to {}".format(src, dest)


def navigate_test_no_graph(src, dest, g):
    print "going from {} to {}".format(src, dest)
    navigate_to_goal_amcl(dest, pi)
    print "finished path from {} to {}".format(src, dest)


def scanner_callback(scanner_data):
    global obstacle_found, goal_failed, retry_obstacle, retry_obstacle_count

    if isinstance(scanner_data, LaserScan):
        # front1 = list(scanner_data.ranges[:20])
        # front2 = list(scanner_data.ranges[-20:])
        front1 = list(scanner_data.ranges[:30])
        front2 = list(scanner_data.ranges[-30:])
        front2.extend(front1)
        is_obs = is_obstacle(front2)
        if is_obs:
            print "scanner_callback obstacle_found"
            obstacle_found = True

        # emergency mode
        front_near = [x for x in front2 if x < obstacle_threshold/2]
        # if len(front_near) > 10:
        if len(front_near) > 15:
            retry_obstacle = True
            retry_obstacle_count += 1
            step_back()
            step_back()
            retry_obstacle = False

            # goal_failed = True

        if retry_obstacle_count > 3:
            goal_failed = True
        # to_the_right_distances = scanner_data.ranges[265: 275]
        # for distance in to_the_right_distances:
        #     if distance_lower_bound < distance < distance_upper_bound:
        #         cmd_vel_pub.publish(stop_vec)
        #         goal_achieved = True


# def camera_callback(image_data):
#     pass
    # if not goal_achieved:
    # cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # cv2.imshow("robot image", cv_image)
    # cv2.waitKey(0)
    # vector = decide_direction(cv_image)
    # cmd_vel_pub.publish(vector)


def amcl_pose_callback(msg):
    global position, orientation
    pose = msg.pose.pose
    position = (pose.position.x, pose.position.y)

    o = pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = euler_from_quaternion(q)
    x, y, z = euler
    orientation = z


def camera_callback(image_data):
    global positive_count, negative_count, lost_the_line, lost_the_line_time, goal_failed

    if not do_line_navigation:
        return
    if obstacle_found:
        return
    if goal_failed:
        return
    if retry_obstacle:
        return
    # if not isinstance(bridge, CvBridge) and isinstance(cmd_vel_pub, rospy.Publisher):
    #     raise
    image = bridge.imgmsg_to_cv2(image_data, 'bgr8')
    h, w, d = image.shape
    # print w
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 10, 10])
    upper_yellow = np.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    search_top = 3*h/4
    # search_bot = 3*h/4 + 20
    search_bot = 3*h/4 + 50
    mask[:search_top, :] = 0
    mask[search_bot:, :] = 0
    mask[:, 0:w/3] = 0  # only look at the line to the right
    M = cv2.moments(mask)
    if M['m00'] == 0:
        # try to find the line
        now = datetime.datetime.now()
        if not lost_the_line:
            lost_the_line = True
            lost_the_line_time = now

        if lost_the_line:
            time_diff = now - lost_the_line_time
            if time_diff.seconds > 8:
                goal_failed = True
                lost_the_line = False

        rotate_right_slowly()
        return
    lost_the_line = False

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # im_circle = cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    # im_circle = cv2.circle(mask, (cx, cy), 20, (0, 0, 255), -1)
    # resized = cv2.resize(im_circle, (w/2, h/2))
    desired = (w/2) + 550
    err = cx - desired
    straight_speed = 0.23
    turn_speed = 0.13
    turn_count_threshold = 5
    # rotation = -(err / 750.0)
    rotation = -(err / 875.0)
    max_theta_speed = 0.8
    min_theta_speed = -0.2

    if rotation > max_theta_speed:  # n right
        rotation = max_theta_speed
    if rotation < min_theta_speed:  # turn left
        rotation = min_theta_speed

    if rotation > 0:  # right
        positive_count += 1
        negative_count = 0
    else:
        negative_count += 1
        positive_count = 0

    if negative_count >= turn_count_threshold or positive_count >= turn_count_threshold:
        # print "turning"
        speed = turn_speed
    else:
        speed = straight_speed

    twist.angular.z = rotation
    twist.linear.x = speed
    # print "right_count: {}, left_count: {}".format(positive_count, negative_count)
    # print twist.angular.z, twist.linear.x
    cmd_vel_pub.publish(twist)
    # cv2.imshow("window", resized)
    # cv2.waitKey(3)


def navigator():
    global cmd_vel_pub, bridge
    bridge = CvBridge()
    rospy.init_node('navigator', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback)
    rospy.Subscriber("/scan", LaserScan, scanner_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    test = False
    if test:
        rospy.spin()
    else:
        g = generate_graph()
        init_move_base()

        if sys.argv[1] == "goto":
            prev = sys.argv[2]
            navigate('start', sys.argv[2], g)
            while True:
                next_ = raw_input("next goal: ")
                navigate(prev, next_, g)
                prev = next_

        goals = get_goals_order()

        # navigate = navigate_test_no_graph
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
