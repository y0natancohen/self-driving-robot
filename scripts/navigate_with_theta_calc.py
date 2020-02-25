#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from dijkstar import Graph, find_path
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped


points = [
    # [x,y]
    ["0", "0"],  # 0 not used
    [0, 0],  # 1
    # [2.15, 0],  # 2
    [2, 0],  # 2
    [2.26, -1.78],  # 3
    [4.4, -1.78],  # 4
    # [2.26, 1.5],  # 5
    [2.26, 1.3],  # 5
    # [5.8, 1.5],  # 6
    [5.5, 1.5],  # 6
    [5.8, 3.25],  # 7
    [7.5, 3.25],  # 8
    # [5.8, -0.75],  # 9
    [5.8, -0.55],  # 9
    [8, -0.75],  # 10
]
q_look_up = (0, 0, 0, 1)
inf_time = rospy.Duration(99999)
move_base = None
# curr_absolute_theta = 0


def generate_graph():
    """
    8     10
    |      |
    7 -6 - 9
       |
       |       4
       |       |
       5 - 2 - 3
           |
           1
    """
    g = Graph()

    add_edge(1, 2, g)
    add_edge(2, 3, g)
    add_edge(3, 4, g)
    add_edge(2, 5, g)
    add_edge(5, 6, g)
    add_edge(6, 7, g)
    add_edge(7, 8, g)
    add_edge(6, 9, g)
    add_edge(9, 10, g)

    return g


def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))


def add_edge(point_num1, point_num2, g):
    d = distance(points[point_num1], points[point_num2])
    g.add_edge(point_num1, point_num2, d)


# def make_goal(goal_num, theta):
def make_goal(goal_num):
    # goal_dict = goals[goal_num]
    x, y = points[goal_num]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    p = Point(x, y, 0)
    # if theta == 0:
    #     q = Quaternion(*q_look_up)
    # else:
    q = quaternion_from_euler(0, 0, 0)
    q = Quaternion(*q)
    goal.target_pose.pose = Pose(p, q)
    return goal


def init_move_base():
    global move_base
    actionlib.ActionClient
    actionlib.ClientGoalHandle
    actionlib.GoalManager
    actionlib.
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()
    move_base.wait_for_server(timeout=inf_time)  # wait to infinity
    move_base.


# def navigate_to_goal_num(goal_num, theta=0.0):
def navigate_to_goal_num(goal_num):
    goal = make_goal(goal_num)
    move_base.send_goal(goal)
    print "going to goal {}...".format(goal_num)
    move_base.wait_for_result(timeout=inf_time)  # wait to infinity
    print "arrived to goal {}!".format(goal_num)
    # result = move_base.wait_for_result(timeout=inf_time)  # wait to infinity
    # print "got result: {}".format(result)
    #
    # state = move_base.get_state()
    # print "state is: {}".format(state)
    # print "result and state: {}".format(result and state)
    print "GoalStatus.SUCCEEDED: {}".format(GoalStatus.SUCCEEDED)


# def calculate_theta_diff(p1, p2):
#     x1, y1 = p1
#     x2, y2 = p2
#     absolute_theta = math.atan((y2 - y1)/(x2 - x1)) - (math.pi/2)
#     return absolute_theta - curr_absolute_theta

#
# def calculate_theta(p1, p2):
#     x1, y1 = p1
#     x2, y2 = p2
#     absolute_theta = math.atan((y2 - y1)/(x2 - x1)) - (math.pi/2)
#     return absolute_theta
#
#
# def update_curr_theta(data):
#     global curr_absolute_theta
#     if isinstance(data, PoseWithCovarianceStamped):
#         pose = data.pose
#         pose = pose.pose
#         o = pose.orientation
#         r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
#         curr_absolute_theta = y
#         print "curr_theta: {}".format(curr_absolute_theta)


def navigator():
    rospy.init_node('navigator', anonymous=True)
    # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, update_curr_theta)
    init_move_base()
    g = generate_graph()
    path = find_path(g, s=1, d=10)
    # prev_node_num = None
    for _i, node_num in enumerate(path.nodes):
        # i = _i + 1  # because I started from 1 and not zero
        # if _i + 1 == len(path.nodes):  # last node, no need to calculate next theta
        #     navigate_to_goal_num(node_num)
        # else:
        #     next_node_num = path.nodes[_i + 1]
        #     # theta_diff = calculate_theta_diff(points[prev_node_num], points[node_num])
        #     theta = calculate_theta(points[next_node_num], points[node_num])
        #     navigate_to_goal_num(node_num, theta=theta)
            navigate_to_goal_num(node_num)
    print "that's all for today"


if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
