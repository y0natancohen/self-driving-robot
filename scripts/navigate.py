#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
from dijkstar import Graph, find_path
import math
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import cv2

cmd_vel_pub = None
bridge = None
obstacle_found = False
pi = math.pi
inf_time = rospy.Duration(99999)
move_base = None
angles = {}

# TODO: fill points coordinates
# TODO: how to use the scan laser to avoid obstacles? (during the way to the goal) maybe changing package configuration
# TODO: stay inside the lines - integrate code from ass2
# TODO: need to stop at a white line before a turn
# TODO: after everything, edit the map and remove current example obstacles
points = {
    # "name": (x, y)
    "start": (3.4, -1.6),
    "start2": (0, 0),
    "pink": (0.25, 2.5),
    "pink2": (0, 0),
    # "green": (-0.35, -0.5),
    "blue": (1.9, -3.8),
    "blue2": (0, 0),
    "red": (-3.4, 2.8),
    "red2": (0, 0),
    "n1": (0, 0),
    "n2": (0, 0),
    "n3": (0, 0),
    "n4": (0, 0),
    "w1": (0, 0),
    "w2": (0, 0),
    "w3": (0, 0),
    "w4": (0, 0),
    "e1": (0, 0),
    "e2": (0, 0),
    "e3": (0, 0),
    "e4": (0, 0),
    "c1": (0, 0),
    "c2": (0, 0),
    "c3": (-0.35, -0.5),
    "c4": (0, 0),
    "s1": (0, 0),
    "s2": (0, 0),
    "s3": (0, 0),
    "s4": (0, 0),
    "nw_out": (0, 0),
    "nw_in": (0, 0),
    "ne_out": (0, 0),
    "ne_in": (0, 0),
    "sw_out": (0, 0),
    "sw_in": (0, 0),
    "se_out": (0, 0),
    "se_in": (0, 0),
}


def generate_graph():
    """ general:
    nw --- n --- ne
    |      |      |
    w ---- c ---- e
    |      |      |
    sw --- s --- se

    junctions- 4-complete graph:
    w1--w2
    |  X |
    w3--w4

    corners- separated:
    nw_o -------
    |
    |   nw_i ---
    |    |

    mid-points:
    |       |
    |       |
    red---red2
    |       |
    |       |
    """
    g = Graph(undirected=False)
    # junction edges
    add_complete_4('n1', 'n2', 'n3', 'n4', g)
    add_complete_4('w1', 'w2', 'w3', 'w4', g)
    add_complete_4('e1', 'e2', 'e3', 'e4', g)
    add_complete_4('s1', 's2', 's3', 's4', g)
    add_complete_4('c1', 'c2', 'c3', 'c4', g)

    # corners edges
    add_edge('n1', 'nw_out', g, pi)
    add_edge('nw_out', 'w1', g, 3*pi/2)
    add_edge('nw_in', 'n3', g, 0)
    add_edge('w2', 'nw_in', g, pi/2)

    add_edge('ne_out', 'n2', g, pi)
    add_edge('e2', 'ne_out', g, pi/2)
    add_edge('n4', 'ne_in', g, 0)
    add_edge('ne_in', 'e1', g, 3*pi/2)

    add_edge('w3', 'sw_out', g, 3*pi/2)
    add_edge('sw_out', 's3', g, 0)
    add_edge('sw_in', 'w4', g, pi/2)
    add_edge('s1', 'sw_in', g, pi)

    add_edge('se_out', 'e4', g, pi/2)
    add_edge('s4', 'se_out', g, 0)
    add_edge('e3', 'se_in', g, 3*pi/2)
    add_edge('se_in', 's2', g, pi)

    # center edges
    add_edge('n3', 'c1', g, 3*pi/2)
    add_edge('c1', 'w2', g, pi)
    add_edge('c2', 'n4', g, pi/2)
    add_edge('e1', 'c2', g, pi)
    add_edge('w4', 'c3', g, 0)
    add_edge('c3', 's1', g, 3*pi/2)
    add_edge('s2', 'c4', g, pi/2)
    add_edge('c4', 'e3', g, 0)

    # mid-points
    # green is c3
    add_edge('se_out', 'start', g, pi/2)
    add_edge('start', 'e4', g, pi/2)

    add_edge('nw_out', 'red', g, 3*pi/2)
    add_edge('red', 'w1', g, 3*pi/2)

    add_edge('c2', 'pink', g, pi/2)
    add_edge('pink', 'n4', g, pi/2)

    add_edge('se_in', 'blue', g, pi)
    add_edge('blue', 's2', g, pi)

    # mid-points across the road
    add_double_edge('start', 'start2', g, pi)
    add_double_edge('red', 'red2', g, 0)
    add_double_edge('pink', 'pink2', g, pi)
    add_double_edge('blue', 'blue2', g, 3*pi/2)

    add_edge('e3', 'start2', g, 3*pi/2)
    add_edge('start2', 'se_in', g, 3*pi/2)

    add_edge('w2', 'red2', g, pi/2)
    add_edge('red2', 'nw_in', g, pi/2)

    add_edge('c2', 'pink2', g, pi/2)
    add_edge('pink2', 'n4', g, pi/2)

    add_edge('s4', 'blue2', g, 0)
    add_edge('blue2', 'se_out', g, 0)

    return g


def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))


def add_edge(p1, p2, g, angle):
    d = distance(points[p1], points[p2])
    g.add_edge(p1, p2, d)
    angles[(p1, p2)] = angle


def add_double_edge(p1, p2, g, angle):
    add_edge(p1, p2, g, angle)
    add_edge(p2, p1, g, (angle + pi) % pi)


def add_complete_4(p1, p2, p3, p4, g):
    add_double_edge(p1, p2, g, 0)
    add_double_edge(p1, p3, g, 3*pi/2)
    add_double_edge(p1, p4, g, 7*pi/4)
    add_double_edge(p2, p3, g, 5*pi/4)
    add_double_edge(p2, p4, g, 3*pi/2)
    add_double_edge(p3, p4, g, 0)


def make_goal(p, angle):
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


def navigate_to_goal(p, angle):
    goal = make_goal(p, angle)
    move_base.send_goal(goal)
    print "going to goal {}...".format(p)
    result = move_base.wait_for_result(timeout=inf_time)  # wait to infinity
    print "got result: {}".format(result)

    state = move_base.get_state()
    print "state is: {}".format(state)
    print "result and state: {}".format(result and state)
    print "GoalStatus.SUCCEEDED: {}".format(GoalStatus.SUCCEEDED)
    print "GoalStatus.aborted: {}".format(GoalStatus.ABORTED)
    print "GoalStatus.active: {}".format(GoalStatus.ACTIVE)
    print "arrived!".format(p)


def _callback_done(*args):
    print ("_callback_done")
    print ("*args: {}".format(args))


def _callback_active(*args):
    print ("_callback_active")
    print ("*args: {}".format(args))


def _callback_feedback(feedback):
    # global obstacle_found
    curr_pose = feedback.base_position.pose
    if not isinstance(curr_pose, Pose):
        raise
    if obstacle_found:
        move_base.cancel_all_goals()
        print("canceled")


# def navigate_to_goal_test(p, angle):
#     # if not isinstance(move_base, actionlib.SimpleActionClient):
#     #     raise
#     goal = make_goal(p, angle)
#     move_base.send_goal(goal, feedback_cb=_callback_feedback)
#     print "going to goal {}...".format(p)
#     move_base.wait_for_result(timeout=inf_time)
#     print "arrived!".format(p)


def get_colors_order():
    """ prgb translates to (pink -> red -> green -> blue) """
    if len(sys.argv) < 2:
        raise Exception('usage: navigate.py "pgrb" (the order of the colors)')
    points_names = {'r': 'red', 'p': 'pink', 'g': 'c3', 'b': 'blue'}
    print(sys.argv[1])
    print([char for char in sys.argv[1]])
    return [points_names[char] for char in sys.argv[1]]


def navigate(src, dest, g):
    path = find_path(g, s=src, d=dest)
    print "planned path from {} to {} is {}".format(src, dest, path)
    edges = [(path.nodes[i], path.nodes[i+1]) for i in range(len(path.nodes[:-1]))]
    for edge in edges:
        angle = angles[edge]
        navigate_to_goal(edge[1], angle)
    print "finished path from {} to {}".format(src, dest)


def navigate_test_no_graph(src, dest, g):
    print "going from {} to {}".format(src, dest)
    navigate_to_goal(dest, pi)
    print "finished path from {} to {}".format(src, dest)


def scanner_callback(scanner_data):
    global obstacle_found

    if isinstance(scanner_data, LaserScan):
        front1 = scanner_data.ranges[:20]
        front2 = scanner_data.ranges[-20:]
        front1_near = [x for x in front1 if x < 0.35]
        front2_near = [x for x in front2 if x < 0.35]
        if len(front1_near) > 0 or len(front2_near) > 0:
            obstacle_found = True
        # to_the_right_distances = scanner_data.ranges[265: 275]
        # for distance in to_the_right_distances:
        #     if distance_lower_bound < distance < distance_upper_bound:
        #         cmd_vel_pub.publish(stop_vec)
        #         goal_achieved = True


def is_yellow(b, g, r):
    return b < 10 and g > 240 and r > 240


def is_white(b, g, r):
    return b > 210 and g > 240 and r > 240


def camera_callback(image_data):
    pass
    # if not goal_achieved:
    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    cv2.imshow("robot image", cv_image)
    cv2.waitKey(0)
    # vector = decide_direction(cv_image)
    # cmd_vel_pub.publish(vector)


def navigator():
    global cmd_vel_pub, bridge
    bridge = CvBridge()

    rospy.init_node('navigator', anonymous=True)
    init_move_base()

    g = generate_graph()
    colors = get_colors_order()

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback)
    rospy.Subscriber("/scan", LaserScan, scanner_callback)

    # testing
    navigate = navigate_test_no_graph
    # testing

    navigate('start', colors[0], g)
    navigate(colors[0], colors[1], g)
    navigate(colors[1], colors[2], g)
    navigate(colors[2], colors[3], g)
    print "that's all for today"


if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
