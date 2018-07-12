#!/usr/bin/env python
import rospy

from tams_ur5_push_execution.srv import *
from geometry_msgs.msg import Pose, Point
from push_predictor import PushPredictor
from lib import content_helper as ch
from math import atan2, pi
import numpy as np

from predictive_sampler_node import get_random_push


def get_normalized_approach_from_box(p, x=0.162, y=0.23, z=0.056, side=None):
    if side == 0:
        p = x * p
    elif side == 1:
        p = x + y * p
    elif side == 2:
        p = x + y + x * p
    elif side == 3:
        p = x + y + x + y * p
    else:
        p = (2 * x + 2 * y) * p

    pose = Pose()
    edge_distance = 0.0
    if(p <= x):
        pose.position.x = min(max(edge_distance, p), x-edge_distance)
        pose.position.y = 0
        pose.orientation = ch.quat_from_yaw(0.5* pi)
    elif (p <= (x + y)):
        pose.position.x = x
        pose.position.y = min(max(edge_distance, p-x ), y-edge_distance)
        pose.orientation = ch.quat_from_yaw(pi)
    elif (p <= (2 * x + y)):
        pose.position.x = min(max(edge_distance, 2 * x + y - p ), x-edge_distance)
        pose.position.y = y
        pose.orientation = ch.quat_from_yaw(1.5* pi)
    else:
        pose.position.x = 0
        pose.position.y = min(max(edge_distance, 2 * (x + y) - p ), y-edge_distance)
        pose.orientation = ch.quat_from_yaw(0.0)

    # adjust to center
    pose.position.x -= 0.5*x;
    pose.position.y -= 0.5*y;
    
    # Pose height is related to box height and tip length
    # By default the tip aligns with the frame of the box.
    # The tip must be lifted in two mutually exclusive cases:
    table_distance = 0.02
    tip_length = 0.08
    # 1. The box is too small and therefore the table distance too short.
    pose.position.z = max(table_distance - 0.5 * z, 0.0)
    # 2. The box is too high for the tip and might touch the gripper
    pose.position.z = max(0.5 * z - tip_length, pose.position.z);
    return pose

def control_to_push(c):
    push = ch.Push()
    push.approach = get_normalized_approach_from_box(c[0])
    push.angle = c[1] - 0.5
    #push.distance = 0.005 + c[2] * 0.025
    #push.distance = 0.01
    push.distance = c[2] * 0.05
    return push

def push_direction_valid(direction, threshold = 0.5):
    for side in [0.0, 0.5, 1.0, 1.5]:
        surface_angle = pi * side
        if abs(surface_angle - direction) < threshold:
            return True
    return False

def sample_directed_control(direction, threshold):
    direction_error = float('inf')
    while direction_error > threshold:
        control = np.random.uniform(0,1,3)
        push = control_to_push(control)
        push_direction = push.angle + ch.get_yaw(push.approach)
        direction_diff = abs(direction - push_direction)
        direction_error = min(direction_diff, 2.0 * pi - direction_diff)
    return control, push

def linalg_dist(p1, p2):
    return np.linalg.norm([p1.position.x - p2.position.x, p1.position.y - p2.position.y])

def se2_pose_distance(p1, p2):
    p_dist = linalg_dist(p1, p2)
    o_dist = abs(ch.get_yaw(p1) - ch.get_yaw(p2)) % (2 * pi)
    o_dist = o_dist if o_dist < pi else 2 * pi - o_dist
    return p_dist + 0.5 * o_dist

def get_cor(p1, p2):
    # get point in the middle of p1 p2
    pm = ch.interpolate_poses(p1, p2)
    pm_yaw = ch.get_yaw(pm)
    pm_tangent_yaw = atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)

    # define perpendicular pose at pm
    pm_cross = Pose()
    pm_cross.position = pm.position
    pm_cross.orientation = ch.quat_from_yaw(pm_tangent_yaw + 0.5 * pi)

    # find diff angle between tangent and pm orientation
    yaw_diff = pm_yaw - pm_tangent_yaw
    p1_tangent_yaw = ch.get_yaw(p1) - yaw_diff

    # helper pose for line creation
    point_x = Pose()
    point_x.position.x = 1.0
    point_x.orientation.w = 1.0

    # define perpendicular pose at p1
    p1_cross = Pose()
    p1_cross.position = p1.position
    p1_cross.orientation = ch.quat_from_yaw(p1_tangent_yaw + 0.5 * pi)

    # create lines along p1 pm -> crossing should be cor
    p1_center_line = ch.get_line(p1.position, ch.transform_pose(p1_cross, point_x).position)
    pm_center_line = ch.get_line(pm.position, ch.transform_pose(pm_cross, point_x).position)

    # find crossing and return cor
    cor_xy = ch.intersection(p1_center_line, pm_center_line)
    cor = False
    if(cor_xy):
        cor = Pose()
        cor.position.x = cor_xy[0]
        cor.position.y = cor_xy[1]
        cor.orientation.w = 1.0
    return cor


def predict_push(req):
    global push_predictor
    res = PredictPushResponse()
    res.success = False

    # get controls
    #print "received", req

    if len(req.control) == 3:

        # create push message from control values (approach, yaw, distance)
        res.next_pose = push_predictor.predict_pose(control_to_push(req.control))
        res.success = True
    else:
        print "Unable to predict push since control vector has invalid size!"

    return res

def steer_push(req):
    global push_predictor
    res = SteerPushResponse()

    min_dist = float('inf')
    best_control = None

    start = Pose()
    start.orientation.w = 1.0
    goal = ch.transform_pose(req.start, req.goal)

    start = req.start
    for i in range(25):
        control = np.random.uniform(0,1,3)
        push = control_to_push(control)
        pose = push_predictor.predict_pose(push)
        dist = se2_pose_distance(ch.transform_pose(req.start, pose), req.goal)
        if best_control is None or dist < min_dist:
            min_dist = dist
            best_control = control

    res.control = best_control
    #res.duration = best_control[2]
    res.duration = linalg_dist(start, goal) / linalg_dist(start, pose)
    #res.success = min_dist < se2_pose_distance(req.start, req.goal)
    res.success = True
    return res

def simple_steer(req, range_rejection=True):
    global push_predictor
    res = SteerPushResponse()
    res.success = False

    start = Pose()
    start.orientation.w = 1.0
    goal = ch.transform_pose(req.start, req.goal)

    # if target is too far off
    start_goal_distance = linalg_dist(start, goal)
    if range_rejection and start_goal_distance > 0.1:
        return res

    goal_threshold = max(0.05, 0.75 * se2_pose_distance(start, goal))

    for i in range(100):

        # sample random push control and predict pose
        res.control = np.random.uniform(0,1,3)
        push = control_to_push(res.control)
        pose = push_predictor.predict_pose(push)

        min_dist = se2_pose_distance(start, goal)
        res.duration = 0

        next_pose = ch.transform_pose(start, pose)
        next_dist = se2_pose_distance(next_pose, goal)

        while next_dist < min_dist:
            res.duration += 1
            min_dist = next_dist
            next_pose = ch.transform_pose(next_pose, pose)
            next_dist = se2_pose_distance(next_pose, goal)

        if res.duration > 0 and min_dist < goal_threshold:
            res.success = True
            print "Found steer control", res.duration
            break

    return res


def simple_steer_with_cor(req, range_rejection=True):
    global push_predictor
    res = SteerPushResponse()
    res.success = False

    start = Pose()
    start.orientation.w = 1.0
    goal = ch.transform_pose(req.start, req.goal)

    # if target is too far off
    start_goal_distance = linalg_dist(start, goal)
    if range_rejection and start_goal_distance > 0.05:
        return res

    # default push direction is straight towards goal
    push_direction = atan2(goal.position.y, goal.position.x)

    # check if we find a good curve around cor
    cor = get_cor(start, goal)
    if cor and linalg_dist(start, cor) < 10 * start_goal_distance:
        cor_phi = atan2(cor.position.y, cor.position.x)

        # flip push direction 'upwards'
        push_direction = cor_phi + 0.5 * pi * (1 if cor.position.x > 0 else -1)

        goal_yaw = ch.get_yaw(goal) % (2 * pi) - pi

        # if the other direction is faster, reverse push angle
        if ( pi - goal_yaw ) * push_direction < 0:
            push_direction = -push_direction

    # if push direction is out of pushable range (+-0.5 rad from surface)
    if not push_direction_valid(push_direction):
        return res


    goal_threshold = 0.1

    for i in range(20):

        # sample random push control and predict pose
        #res.control = np.random.uniform(0,1,3)
        #push = control_to_push(res.control)
        control, push = sample_directed_control(push_direction, 0.1)
        pose = push_predictor.predict_pose(push)

        min_dist = se2_pose_distance(start, goal)
        res.duration = 0

        next_pose = ch.transform_pose(start, pose)
        next_dist = se2_pose_distance(next_pose, goal)

        while next_dist < min_dist:
            res.duration += 1
            min_dist = next_dist
            next_pose = ch.transform_pose(next_pose, pose)
            next_dist = se2_pose_distance(next_pose, goal)

        if res.duration > 0 and min_dist < goal_threshold:
            res.success = True
            res.control = control
            print "Found steer control", res.duration
            break

    return res


def steer_with_cor(req):
    global push_predictor
    res = SteerPushResponse()
    res.success = False

    start = req.start
    goal = req.goal

    goal_threshold = 0.05

    for i in range(100):

        # sample random push control and predict pose
        control = np.random.uniform(0,1,3)
        push = control_to_push(control)
        pose = push_predictor.predict_pose(push)

        # get cor of sampled pose or continue
        cor = get_cor(start, ch.transform_pose(start,pose))
        if not cor:
            continue

        radius = linalg_dist(start, cor)
        cor_to_goal = ch.get_diff_pose(cor, goal)

        # check for goal distance
        dist_cor_goal = np.linalg.norm([cor_to_goal.position.x, cor_to_goal.position.y])
        goal_distance_error = abs(radius - dist_cor_goal)

        # check sampled push might be a candidate
        if goal_distance_error < goal_threshold:
            cor_to_start = ch.get_diff_pose(cor, start)
            start_yaw = ch.get_yaw(cor_to_start)
            start_tang_yaw = atan2(cor_to_start.position.y, cor_to_start.position.x) + 0.5*pi

            # difference of pose rotation from tangent
            diff_yaw = start_tang_yaw - start_yaw

            # compute best target candidate
            target = Pose()
            target.position.x = radius * cor_to_goal.position.x / dist_cor_goal
            target.position.y = radius * cor_to_goal.position.y / dist_cor_goal
            target_tang_yaw = atan2(target.position.y, target.position.x) + 0.5*pi
            target_yaw = target_tang_yaw - diff_yaw

            goal_yaw_error = abs(target_yaw - ch.get_yaw(cor_to_goal))

            # check if we found a solution
            if ( goal_distance_error + 0.5 * goal_yaw_error ) < goal_threshold:
                print "Found new steer control"
                res.control = control
                single_push_yaw = ch.get_yaw(pose)
                rotate_positive = single_push_yaw > 0.0

                # test if we need angle wrapping
                if rotate_positive and target_yaw < start_yaw:
                    target_yaw += 2 * pi
                elif not rotate_positive and target_yaw > start_yaw:
                    target_yaw -= 2 * pi

                # duration is computed by dividing the complete diff angle by step angle
                res.duration = abs(target_yaw - start_yaw) / ch.get_yaw(pose)
                res.success = True
                break
    return res

def random_pose():
    p = Pose()
    p.position.x = np.random.uniform(-0.3, 0.3)
    p.position.y = np.random.uniform(-0.3, 0.3)
    p.orientation = ch.quat_from_yaw(np.random.uniform(0.0, 2*pi))
    return p

def steer_benchmark(iterations):
    import time

    req = SteerPushRequest()

    #start_time = time.time()
    #solutions = 0
    #for i in range(iterations):
    #    req.start = random_pose()
    #    req.goal = random_pose()
    #    if(steer_with_cor(req).success):
    #        solutions += 1

    #print "steer_with_cor:"
    #print " time:", (time.time() - start_time), "s"
    #print " solutions:", solutions

    start_time = time.time()
    solutions = 0
    for i in range(iterations):
        req.start = random_pose()
        req.goal = random_pose()
        if(simple_steer(req, True).success):
            solutions += 1

    print "simple_steer:"
    print " time:", (time.time() - start_time), "s"
    print " solutions:", solutions

def prediction_server():
    global push_predictor
    push_predictor = PushPredictor(True)
    rospy.init_node('prediction_node');
    push_service = rospy.Service('predict_push_service', PredictPush, predict_push)
    #steer_service = rospy.Service('steer_push_service', SteerPush, steer_push)
    steer_service = rospy.Service('steer_push_service', SteerPush, simple_steer)
    print "Ready to predict pushes"
    rospy.spin()

if __name__=="__main__":
    print "init predictive sampler node"
    prediction_server()
