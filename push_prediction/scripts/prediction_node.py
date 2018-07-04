#!/usr/bin/env python
import rospy

from tams_ur5_push_execution.srv import *
from geometry_msgs.msg import Pose
from push_predictor import PushPredictor
from lib import content_helper as ch
import math


def get_normalized_approach_from_box(p, x=0.162, y=0.23, z=0.056):
    pose = Pose()
    edge_distance = 0.0
    if(p <= x):
        pose.position.x = min(max(edge_distance, p), x-edge_distance)
        pose.position.y = 0
        pose.orientation = ch.quat_from_yaw(0.5* math.pi)
    elif (p <= (x + y)):
        pose.position.x = x
        pose.position.y = min(max(edge_distance, p-x ), y-edge_distance)
        pose.orientation = ch.quat_from_yaw(math.pi)
    elif (p <= (2 * x + y)):
        pose.position.x = min(max(edge_distance, 2 * x + y - p ), x-edge_distance)
        pose.position.y = y
        pose.orientation = ch.quat_from_yaw(1.5* math.pi)
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


def predict_push(req):
    global push_predictor
    res = PredictPushResponse()
    res.success = False

    # get controls
    c = req.control
    print "received", req

    if len(c) == 3:

        # create push message from control values (approach, yaw, distance)
        push = ch.Push()
        push.approach = get_normalized_approach_from_box(c[0])
        push.angle = c[1] - 0.5
        push.distance = c[2] * 0.03

        res.next_pose = push_predictor.predict_pose(push)
        res.success = True

    else:
        print "Unable to predict push since control vector has invalid size!"

    return res

def prediction_server():
    global push_predictor
    push_predictor = PushPredictor()
    rospy.init_node('prediction_node');
    s = rospy.Service('predict_push_service', PredictPush, predict_push)
    print "Ready to predict pushes"
    rospy.spin()

if __name__=="__main__":
    print "init predictive sampler node"
    prediction_server()
