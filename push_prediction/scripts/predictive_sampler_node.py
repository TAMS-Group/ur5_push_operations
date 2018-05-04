#!/usr/bin/env python

import rospy
from tams_ur5_push_execution.srv import *
from geometry_msgs.msg import Pose

from lib import content_helper as ch
import math
import numpy as np

from simple_regression import PushPredictor


global push_predictor

def get_pos_cost(pose, target):
    diff_pose = ch.get_diff_pose(pose, target)
    distance = math.sqrt(diff_pose.position.x**2 + diff_pose.position.y**2)
    yaw = ch.get_yaw(diff_pose)
    return math.sqrt(distance**2 + yaw**2)


def get_random_push():
    push = ch.Push()
    push.approach = sample_random_pose_from_box(0.162, 0.23, 0.056)
    push.angle = np.random.uniform(-0.5, 0.5)
    push.distance = 0.03
    return push
    

def sample_random_pose_from_box(x, y, z):
    pose = Pose()
    p = np.random.uniform(0.0, 2 * (x + y))
    if(p <= x):
        pose.position.x = p
        pose.position.y = 0
        pose.orientation = ch.quat_from_yaw(0.5* math.pi)
    elif (p <= (x + y)):
        pose.position.x = x
        pose.position.y = p - x
        pose.orientation = ch.quat_from_yaw(math.pi)
    elif (p <= (2 * x + y)):
        pose.position.x = 2 * x + y - p
        pose.position.y = y
        pose.orientation = ch.quat_from_yaw(1.5* math.pi)
    else:
        pose.position.x = 0
        pose.position.y = 2 * (x + y) - p
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


def sample_push(start_pose, target_pose):
    global push_predictor
    # find out current cost
    start_cost = get_pos_cost(start_pose, target_pose)

    # try to sample 10 solutions within 1000 attempts
    solutions = []
    attempts = 0
    while( len(solutions) < 10 and attempts < 1000):

        # sample push and predict pose and cost
        push = get_random_push()
        pose = push_predictor.predict_pose(push)
        cost = get_pos_cost(pose, target_pose)
    
        # if cost is better then current, add to solutions
        if(cost < start_cost):
            solutions.append((cost, push, pose))
        attempts+=1

    # sort by cost and return best solution or None if empty
    solutions = sorted(solutions, key=lambda sol: sol[0])
    print solutions[0][0], solutions[0][1].approach 
    return solutions[0][1] if len(solutions) > 0 else None


def sample_predictive_push(req):
    print "Recieve sample request"
    res = SamplePredictivePushResponse()
    res.success = False

    # try to sample push
    push = sample_push(req.object_pose, req.target)
    if push is not None:
        #fill push
        res.push.mode = 0
        res.push.distance = push.distance
        #fill push approach
        res.push.approach.frame_id = req.object_id
        res.push.approach.point = push.approach.position
        res.push.approach.normal = push.approach.orientation
        res.push.approach.angle = push.angle
        #set successful
        res.success = True

    return res
	

def predictive_push_sampler_server():
    global push_predictor
    push_predictor = PushPredictor()
    rospy.init_node('predictive_push_sampler_node');
    s = rospy.Service('predictive_push_sampler', SamplePredictivePush, sample_predictive_push)
    print "Ready to sample pushes"
    rospy.spin()

if __name__=="__main__":
    print "init predictive sampler node"
    predictive_push_sampler_server()
