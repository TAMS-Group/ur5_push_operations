#!/usr/bin/env python

import numpy as np
from tf import transformations as t
import geometry_msgs.msg
from shapely.geometry import box, LinearRing, LineString
from shapely import affinity
import math
import matplotlib.pyplot as plt

# own helper modules
from lib import load_samples
from lib.content_helper import *


def plot_push_results(pushes, poses):
    bins = np.arange(-0.5125, 0.5375, 0.025)
    target_angles = np.arange(-0.5, 0.525, 0.025)
    bin_map = np.digitize([push.angle for push in pushes], bins)
    candidates = [[a, [], []] for a in target_angles]
    for i, (push, pose) in enumerate(zip(pushes, poses)):
        d = bin_map[i] - 1
        ta = target_angles[d]
        candidates[d][1].append(push)
        candidates[d][2].append(pose)
        #if((candidates[d][1] is None) or abs(push.angle - ta) < abs(candidates[d][1].angle - ta)):
        #    candidates[d] = [ta, push, pose]


    t_boxes = []
    for c in candidates:
        target_angle = c[0]
        c_pushes = c[1]
        c_poses = c[2]
        if len(c_poses) > 0:

            x = np.median([p.position.x for p in c_poses])
            y = np.median([p.position.y for p in c_poses])

            push_angle = np.median([p.angle for p in c_pushes])
            push_x = np.median([p.approach.position.x for p in c_pushes])
            push_y = np.median([p.approach.position.y for p in c_pushes])

            yaw = np.median(np.array([get_yaw(p) for p in c_poses]))
            cos_a = math.cos(yaw)
            sin_a = math.sin(yaw)

            transformed_box = affinity.affine_transform(get_box(0.162, 0.23), [cos_a, -sin_a, sin_a, cos_a, x, y])
            push_line = get_line(push_x, push_y, c_pushes[0].distance - 0.004, push_angle + get_yaw(push.approach))
            t_boxes.append((target_angle, transformed_box, push_line))
        

    box = get_box(0.162, 0.23)
    #x, y = box.xy
    y, x = LinearRing(box.exterior.coords).xy
    plt.plot(x, y, color="black", linewidth=3)

    for angle, box, push_line in t_boxes:
        y, x = LinearRing(box.exterior.coords).xy
        plt.plot(x, y)
        y,x = push_line.xy
        plt.plot(x, y)
        

    plt.xlim(-0.2, 0.2)
    plt.ylim(-0.2, 0.2)
    plt.axes().set_aspect('equal', adjustable='box')
    plt.show()

def get_box(x, y):
    return box(-0.5*x, -0.5*y, 0.5*x, 0.5*y)

def get_line(x, y, length, angle):
    return LineString([(x, y), (x+length*math.cos(angle), y+length*math.sin(angle))])

if __name__=="__main__":
    print "plot pushes"
    pre_poses, post_poses, diff_poses, pushes = load_samples.load_samples("dataset1")

    yaws = [(i, get_yaw(pose)) for i,pose in diff_poses]
    distances = [(i, np.linalg.norm([pose.position.x, pose.position.y])) for i,pose in diff_poses]


    def push_cond(push):
        return abs(push.approach.position.y - 0.03) < 0.005 and push.approach.position.x == -0.081
    def pose_cond(pose):
        return abs(pose.position.x)+abs(pose.position.z) > 0.015 or get_yaw(pose) > 0.01

    selection = [push.id for push,pose in zip(pushes,diff_poses) if push_cond(push) and pose_cond(pose[1])]

    distances = [d for d in distances if d[0] in selection]
    yaws = [y for y in yaws if y[0] in selection]
    #push_vec = [push for i,push in push_vec if i in selection]
    pushes = [push for push in pushes if push.id in selection]

    #push_vec = [(push.id, [push.approach.position.x, push.approach.position.y, get_yaw(push.approach), push.angle]) for push in pushes]

    plot_push_results(pushes, [pose for i,pose in diff_poses if i in selection])
