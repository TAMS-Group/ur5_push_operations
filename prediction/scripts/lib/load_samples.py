#!/usr/bin/env python

from geometry_msgs.msg import Pose, Quaternion

from content_helper import *

def load_csv(directory, filename):
    header = None
    data = []
    with open(directory + "/" + filename+".csv", 'r') as f:
        for i, line in enumerate(f):
            if(i==0):
                header = line.strip().split(",")
            else:
                data.append(line.strip().split(","))
	return header, data

def csv_to_pose(line):
    pose = Pose()
    pose.position.x = float(line[0])
    pose.position.y = float(line[1])
    pose.position.z = float(line[2])
    pose.orientation.x = float(line[3])
    pose.orientation.y = float(line[4])
    pose.orientation.z = float(line[5])
    pose.orientation.w = float(line[6])
    return pose

def csv_to_push(line):
    push = Push()
    push.id = get_csv_line_id(line)
    push.approach = csv_to_pose(line[3:])
    push.angle = float(line[-2])
    push.distance = float(line[-1])
    return push

def get_csv_line_id(line):
    return int(line[0]) if isinstance(line, list) and len(line) > 0 and line[0].isdigit() else -1

def load_samples(directory):
    pre_poses_header, pre_poses_csv = load_csv(directory, "pre_poses")
    post_poses_header, post_poses_csv = load_csv(directory, "post_poses")
    pushes_header, pushes_csv = load_csv(directory, "pushes")

    pre_poses = [(get_csv_line_id(line), csv_to_pose(line[1:])) for line in pre_poses_csv]
    post_poses = [(get_csv_line_id(line), csv_to_pose(line[1:])) for line in post_poses_csv]
    diff_poses = [(i,get_diff_pose(p1, p2)) if i==j else None for ((i,p1), (j,p2)) in zip(pre_poses, post_poses)]
    pushes = [csv_to_push(line) for line in pushes_csv]

    return pre_poses, post_poses, diff_poses, pushes
