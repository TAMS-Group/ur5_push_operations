#!/usr/bin/env python

import rospy
from tams_ur5_push_msgs.msg import PushTrajectory 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from graph_msgs.msg import GeometryGraph, Edges

from lib.marker_helper import init_marker, init_graph_markers


dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112


def visualize_object_trajectory(traj_msg):
    global marker_pub, marker_array_pub, visible_markers, visible_marker_arrays
    markers = MarkerArray()

    # visualize as graph
    nodes = [p.position for p in traj_msg.poses]
    edges = [Edges([i, i+1], [1.0, 1.0]) for i in range(len(nodes)-1)]
    edges.append(Edges())
    color = ColorRGBA(0.0,1.0,0.0,1.0)
    #visualize_graph(nodes, edges, nodes_id=2, edges_id=3, nodes_color=color, edges_color=color, linewidth=0.003, is_path=True)
    pose = Pose()
    pose.position.z = 0.5*dim_Z
    pose.orientation.w = 1.0
    points, lines = init_graph_markers(nodes, edges, nodes_id=2, edges_id=3, pose=pose, nodes_color=color, edges_color=color, linewidth=0.005, is_path=True)
    marker_pub.publish(points)
    marker_pub.publish(lines)
    visible_markers.append(points)
    visible_markers.append(lines)

    # visualize object path with markers
    # TODO: we might use the existing marker to copy the actual shape instead of using hard coded values
    for i in range(len(traj_msg.poses))[0::10]:
        pose = traj_msg.poses[i]
        progress = float(i) / len(traj_msg.poses)
        color = ColorRGBA(progress,0.0,1-progress,0.25)
        scale = Vector3(dim_X, dim_Y, dim_Z)
        pose.position.z = 0.5*dim_Z
        markers.markers.append(init_marker(i, Marker.CUBE, scale=scale, pose=pose, color=color))

    marker_array_pub.publish(markers)
    visible_marker_arrays.append(markers)

def visualize_planner_data(graph_msg):
    global marker_pub, visible_markers
    #visualize_graph(graph_msg.nodes, graph_msg.edges)
    pose = Pose()
    pose.position.z = 0.5*dim_Z
    pose.orientation.w = 1.0

    points, lines = init_graph_markers(graph_msg.nodes, graph_msg.edges, pose=pose)

    marker_pub.publish(points)
    marker_pub.publish(lines)
    visible_markers.append(points)
    visible_markers.append(lines)

def remove_all_markers():
    global marker_pub, marker_array_pub, visible_markers, visible_marker_arrays
    for marker in visible_markers:
        marker.action = Marker.DELETE
        marker_pub.publish(marker)

    for marker_array in visible_marker_arrays:
        for i in range(len(marker_array.markers)):
            marker_array.markers[i].action = Marker.DELETE
        marker_array_pub.publish(marker_array)

    visible_markers = []
    visible_marker_arrays = []


def init_subscribers():
    traj_sub = rospy.Subscriber('/push_trajectory', PushTrajectory, visualize_object_trajectory, queue_size=1)
    data_sub = rospy.Subscriber('/push_planner_graph', GeometryGraph, visualize_planner_data, queue_size=1)

def init_publishers():
    global marker_pub, marker_array_pub, visible_markers, visible_marker_arrays
    visible_markers = []
    visible_marker_arrays = []
    marker_array_pub = rospy.Publisher("/push_trajectory_markers", MarkerArray, queue_size=1)
    marker_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=1)

if __name__=="__main__":
    print "init push planner visualization node"
    rospy.init_node('push_trajectory_visualization_node');
    init_subscribers()
    init_publishers()
    print "Ready to visualize push trajectories"
    rospy.spin()
