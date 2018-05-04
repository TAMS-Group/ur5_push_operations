#!/usr/bin/env python

import numpy as np

import itertools

from sklearn import linear_model, neural_network
from sklearn.externals import joblib

# own helper modules
from lib import load_samples
from lib.content_helper import *

import glob

from math import *

import matplotlib.pyplot as plt

def train_model(X, Y, model='neural'):
    if model=='neural':
        reg = neural_network.MLPRegressor()
    else:
        reg = linear_model.LinearRegression()
    reg.fit(X, Y)

    errors = []
    for x, y in zip(X, Y):
        errors.append(abs(y - reg.predict([x])[0]))

    return reg, errors

def plot_pushes(pushes, values):
    def push_cond(push):
        #return abs(push.approach.position.y)<0.005
	#return abs(push.approach.position.y)<0.005
	return True

        
    p_vec = [push.angle for push in pushes if push_cond(push)]
    p_ids = [push.id for push in pushes if push_cond(push)]
    y_vec = [value for (value_id, value) in values if value_id in p_ids]
    v_ids = [value_id for (value_id, value) in values if value_id in p_ids]

    plt.plot(p_vec, y_vec, 'o')


def normalize_push(push):
    x = (0.081 + push.approach.position.x ) / 0.162
    y = (0.115 + push.approach.position.y ) / 0.23
    yaw = (get_yaw(push.approach) % (2 * pi)) / (2 * pi)
    push_angle = (push.angle % pi) / pi
    return [x, y, yaw, push_angle]

def create_pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation = quat_from_yaw(yaw)
    return pose

def create_push(i, vec):
    push = Push()
    push.approach = create_pose(vec[0], vec[1], vec[2])
    push.angle = vec[3]
    push.distance = 0.03
    return push

def plot_predictions(push_x, push_y, push_yaw, x_model, y_model, yaw_model):
    pushes = [create_push(i,[push_x, push_y, push_yaw, np.random.uniform(-0.5, 0.5)]) for i in range(500)]

    push_vecs = [normalize_push(push) for push in pushes]

    x_vals = x_model.predict(push_vecs)
    y_vals = y_model.predict(push_vecs)
    yaws = yaw_model.predict(push_vecs)

    poses = [create_pose(x, y, yaw-0.5) for x,y,yaw in zip(x_vals,y_vals,yaws)]

    import plot_pushes
    plot_pushes.plot_push_results(pushes, poses)


def initialize_models():
    pre_poses, post_poses, diff_poses, pushes = load_samples.load_samples("dataset1")

    push_vec = [(push.id, normalize_push(push)) for push in pushes]
    yaws = [(i, get_yaw(pose)) for i,pose in diff_poses]
    distances = [(i, np.linalg.norm([pose.position.x, pose.position.y])) for i,pose in diff_poses]
    xvals = [(i, pose.position.x) for i,pose in diff_poses]
    yvals = [(i, pose.position.y) for i,pose in diff_poses]

    push_x = 0.00
    push_y = 0.115
    push_yaw = 0.5*pi

    def push_cond(push):
        return True
        #return abs(push.approach.position.y - push_y) < 0.02 and push.approach.position.x == -0.081
    def pose_cond(pose):
        return abs(pose.position.x)+abs(pose.position.z) > 0.015 or get_yaw(pose) > 0.01
	

    skip_condition = False
    selection = [push.id for push,pose in zip(pushes,diff_poses) if skip_condition or (push_cond(push) and pose_cond(pose[1]))]

    distances = [d for d in distances if d[0] in selection]
    xvals = [x for x in xvals if x[0] in selection]
    yvals = [y for y in yvals if y[0] in selection]
    yaws = [y for y in yaws if y[0] in selection]
    push_vec = [push for i,push in push_vec if i in selection]
    pushes = [push for push in pushes if push.id in selection]

    yaw_model, yaw_errors = train_model(push_vec, [(yaw + 0.5) for _,yaw in yaws])

    max_distance = np.max([d for _,d in distances])
    max_xval = np.max([x for _,x in xvals])
    max_yval = np.max([y for _,y in yvals])

    distance_model, distance_errors = train_model(push_vec, [distance/max_distance for _,distance in distances])

    x_model, x_errors = train_model(push_vec, [x for _,x in xvals], 'linear')

    y_model, y_errors = train_model(push_vec, [y for _,y in yvals], 'linear')

    return x_model, y_model, distance_model, yaw_model

class PushPredictor(object):

    def __init__(self):
        x_model_file = 'x_model.pkl'
        y_model_file = 'y_model.pkl'
        yaw_model_file = 'yaw_model.pkl'
        dump_files = glob.glob('*.pkl')
        if all(f in dump_files for f in [x_model_file, y_model_file, yaw_model_file]):
            print "Model dumps found - loading x/y/yaw models"
            x_model = joblib.load(x_model_file)
            y_model = joblib.load(y_model_file)
            yaw_model = joblib.load(yaw_model_file)
        else:
            print "No model dumps found - initializing new ones and dumping to files for later sessions."
            x_model, y_model, _, yaw_model = initialize_models()
            joblib.dump(x_model, x_model_file)
            joblib.dump(y_model, y_model_file)
            joblib.dump(yaw_model, yaw_model_file)
            
        self.x_model = x_model
        self.y_model = y_model
        self.yaw_model = yaw_model
        print "Predictor ready!"

    def predict_next_pose(self, push, start_pose):
        prediction = self.predict_pose(push)
        return transform_pose(start_pose, prediction)

    def predict_pose(self, push):
        push_vec = normalize_push(push)
        x = self.x_model.predict([push_vec])[0]
        y = self.y_model.predict([push_vec])[0]
        yaw = self.yaw_model.predict([push_vec])[0]
        return create_pose(x, y, yaw-0.5)

############### main
if __name__=="__main__":
    pre_poses, post_poses, diff_poses, pushes = load_samples.load_samples("dataset1")

    push_vec = [(push.id, normalize_push(push)) for push in pushes]
    yaws = [(i, get_yaw(pose)) for i,pose in diff_poses]
    distances = [(i, np.linalg.norm([pose.position.x, pose.position.y])) for i,pose in diff_poses]
    xvals = [(i, pose.position.x) for i,pose in diff_poses]
    yvals = [(i, pose.position.y) for i,pose in diff_poses]

    push_x = 0.00
    push_y = 0.115
    push_yaw = 0.5*pi

    def push_cond(push):
        return True
        return abs(push.approach.position.y - push_y) < 0.02 and push.approach.position.x == -0.081
    def pose_cond(pose):
        return abs(pose.position.x)+abs(pose.position.z) > 0.015 or get_yaw(pose) > 0.01
	

    skip_condition = False
    selection = [push.id for push,pose in zip(pushes,diff_poses) if skip_condition or (push_cond(push) and pose_cond(pose[1]))]
    print len(selection)

    distances = [d for d in distances if d[0] in selection]
    xvals = [x for x in xvals if x[0] in selection]
    yvals = [y for y in yvals if y[0] in selection]
    yaws = [y for y in yaws if y[0] in selection]
    push_vec = [push for i,push in push_vec if i in selection]
    pushes = [push for push in pushes if push.id in selection]


    yaw_model, yaw_errors = train_model(push_vec, [(yaw + 0.5) for _,yaw in yaws])
    print "yaws", np.median(yaw_errors), np.mean(yaw_errors)

    max_distance = np.max([d for _,d in distances])
    max_xval = np.max([x for _,x in xvals])
    max_yval = np.max([y for _,y in yvals])

    distance_model, distance_errors = train_model(push_vec, [distance/max_distance for _,distance in distances])
    print "distances", np.median(distance_errors), np.median(distance_errors)

    x_model, x_errors = train_model(push_vec, [x for _,x in xvals], 'linear')
    print "x", np.median(x_errors), np.median(x_errors)

    y_model, y_errors = train_model(push_vec, [y for _,y in yvals], 'linear')
    print "y", np.median(y_errors), np.median(y_errors)

    #plot_pushes(pushes, distances)
    #plot_pushes(pushes, yaws)
    #plt.show()
    plot_predictions(push_x, push_y, push_yaw, x_model, y_model, yaw_model)
