#!/usr/bin/env python
import rospy
import numpy
import Queue
import roslib
roslib.load_manifest('nao_basic')
from nao_basic.msg import pose, action, confirmation
from std_msgs.msg import String
from math import sqrt, cos, sin


R4 = [30.0,  30.0]
C1 = [30.0, 120.0]
R1 = [30.0, 200.0]

R5 = [150.0,  30.0]
C2 = [150.0, 120.0]
R2 = [150.0, 200.0]

R6 = [230.0,  30.0]
C3 = [230.0, 120.0]
R3 = [230.0, 200.0]

cells   = [R4, C1, R1, R5, C2, R2, R6, C3, R3]
cells_n = ['R4', 'C1', 'R1', 'R5', 'C2', 'R2', 'R6', 'C3', 'R3']

up   = [C1, R1, None, C2, R2, None, C3, R3, None]
up_n = ['C1', 'R1', 'None', 'C2', 'R2', 'None', 'C3', 'R3', 'None']

right   = [R5, C2, R2, R6, C3, R3, None, None, None]
right_n = ['R5', 'C2', 'R2', 'R6', 'C3', 'R3', 'None', 'None', 'None']

left    = [None, None, None, R4, C1, R1, R5, C2, R2]
left_n  = ['None', 'None', 'None', 'R4', 'C1', 'R1', 'R5', 'C2', 'R2']

down    = [None, R4, C1, None, R5, C2, None, R6, C3]
down_n  = ['None', 'R4', 'C1', 'None', 'R5', 'C2', 'None', 'R6', 'C3']

def identify_cell(pose):
    dist_v = [distance(pose, cells[i]) for i in range(9)]
    # dist_v = numpy.array([distance(pose, cells[i]) for i in range(9)])
    min_index = numpy.argmin(dist_v)
    print "the cell where the NAO is:", cells_n[min_index]
    return min_index

def chatterCallback(data):
    print "received request from bt action: move", data.data
    global raw_pose

    pose = median_filter(raw_pose) # median_filter the position
    print "filtered_position", pose
    cell_index = identify_cell(pose)
    plan_goal = [0.0, 0.0]

    if data.data == "up":
        print "I'm going to cell (up)   :", up_n[cell_index]
        plan_goal = up[cell_index]
    if data.data == "down":
        print "I'm going to cell (down) :", down_n[cell_index]
        plan_goal = down[cell_index]
    if data.data == "left":
        print "I'm going to cell (left) :", left_n[cell_index]
        plan_goal = left[cell_index]
    if data.data == "right":
        print "I'm going to cell (right):", right_n[cell_index]
        plan_goal = right[cell_index]

    relative_x = plan_goal[0] - pose[0]
    relative_y = plan_goal[1] - pose[1]
    relative_pose = [relative_x, relative_y]
    oriented_relative_pose = rotate_2d_vector(relative_pose, - pose[2])

    action_pub = rospy.Publisher('next_move', action)
    next_action = action()
    next_action.type = 'goto'
    next_action.x = oriented_relative_pose[0]
    next_action.y = oriented_relative_pose[1]
    # info = 'move in X: %s,  Y: %s' %(next_action.x,next_action.y)
    # rospy.loginfo(info)
    print "publishing to the ros server:", next_action.x, next_action.y
    action_pub.publish(next_action)

def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)) # cm

def poseCallback(data): # data = '(x, y, theta)' x, y in cm, theta in radians
    global raw_pose
    while raw_pose.qsize() < 5:
        print "while loop"
    	raw_pose.put([data.x, data.y, data.theta])
        print "inserting to FIFO queue", [data.x, data.y, data.theta]
    else:
        print "else"
        raw_pose.get()
    	raw_pose.put([data.x, data.y, data.theta])
        print "inserting to FIFO queue", [data.x, data.y, data.theta]
    print "qsize", raw_pose.qsize()

def median_filter(raw_pose):
    list_pose = list(raw_pose.queue)
    print type(list_pose)
    print type(raw_pose)
    # median_x
    x_list = [pose[0] for pose in list_pose]
    median_x = numpy.median(x_list)
    # median_y
    y_list = [pose[1] for pose in list_pose]
    median_y = numpy.median(y_list)
    # median_theta
    theta_list = [pose[2] for pose in list_pose]
    median_theta = numpy.median(theta_list)
    return [median_x, median_y, median_theta]

def confirmCallback(data):
    global confirm
    name = data.name
    done = data.done
    confirm = [name, done]

def check_dist(cur_pose, waypoint, precision=20):
    dist = distance(cur_pose, waypoint)
    if (dist<=precision):
        return True
    return False

def rotate_2d_vector(v, theta):
    new_v = [0, 0]
    new_v[0] = cos(theta)*v[0] - sin(theta)*v[1]
    new_v[1] = sin(theta)*v[0] + cos(theta)*v[1]
    return new_v

def planner():
    global raw_pose
    rospy.init_node('planner')
    raw_pose = Queue.Queue(5)
    rospy.Subscriber('cur_pose', pose, poseCallback)
    rospy.Subscriber('action_done', confirmation, confirmCallback)
    rospy.Subscriber("chatter", String, chatterCallback)
    # action_pub = rospy.Publisher('next_move', action)
    rospy.spin()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
