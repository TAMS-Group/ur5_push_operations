#!/usr/bin/env python

import rospy
from tams_ur5_push_msgs.msg import PushTrajectory 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import *
from graph_msgs.msg import GeometryGraph, Edges

from lib.marker_helper import init_marker, init_graph_markers


dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112


def visualize_object_trajectory(traj_msg):
    global marker_pub

    # visualize as graph
    nodes = [p.position for p in traj_msg.poses]
    edges = [Edges([i, i+1], [1.0, 1.0]) for i in range(len(nodes)-1)]
    edges.append(Edges())
    color = ColorRGBA(0.0,1.0,0.0,1.0)
    #visualize_graph(nodes, edges, nodes_id=2, edges_id=3, nodes_color=color, edges_color=color, linewidth=0.003, is_path=True)
    pose = Pose(Point(0,0,0.5*dim_Z), Quaternion(0,0,0,1))
    points, lines = init_graph_markers(nodes, edges, nodes_id=2, edges_id=3, pose=pose, nodes_color=color, edges_color=color, linewidth=0.005, is_path=True)
    marker_pub.publish(points)
    marker_pub.publish(lines)

    visualize_object_markers(traj_msg.poses)


# visualize object path with markers
# TODO: we should use the existing marker to copy the actual shape instead of using hard coded values
def visualize_object_markers(poses, step_size=10):
    global marker_array_pub
    markers = MarkerArray()
    for step, pose in list(enumerate(poses))[0::step_size]:
            pose.position.z = 0.5*dim_Z
            progress = float(step) / len(poses)
            color = ColorRGBA(progress,0.0,1-progress,0.25)
            scale = Vector3(dim_X, dim_Y, dim_Z)
            markers.markers.append(init_marker(step, Marker.CUBE, scale=scale, pose=pose, color=color))

    marker_array_pub.publish(markers)


def visualize_planner_data(graph_msg):
    global marker_pub
    #visualize_graph(graph_msg.nodes, graph_msg.edges)
    pose = Pose(Point(0,0,0.5*dim_Z), Quaternion(0,0,0,1))

    points, lines = init_graph_markers(graph_msg.nodes, graph_msg.edges, pose=pose)

    marker_pub.publish(points)
    marker_pub.publish(lines)


def remove_all_markers():
    global marker_pub, marker_array_pub
    # clear visualized markers
    delete_all = Marker(action=Marker.DELETEALL)
    marker_pub.publish(delete_all)
    marker_array_pub.publish(MarkerArray(markers=[delete_all]))


def init_publishers():
    global marker_pub, marker_array_pub
    marker_array_pub = rospy.Publisher("/push_trajectory_markers", MarkerArray, queue_size=1)
    marker_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=1)
