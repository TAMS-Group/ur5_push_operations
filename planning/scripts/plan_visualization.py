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

class PlanVisualization:

    def __init__(self):
        #init publishers
        self.marker_array_pub = rospy.Publisher("/push_trajectory_markers", MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=1)

    # visualize trajectory as graph
    def visualize_trajectory(self, traj_msg, show_object=True):
        nodes = [p.position for p in traj_msg.poses]
        edges = [Edges([i, i+1], [1.0, 1.0]) for i in range(len(nodes)-1)]
        edges.append(Edges()) # add empty Edges() for last node
        color = ColorRGBA(0.0,1.0,0.0,1.0)
        pose = Pose(Point(0,0,0.5*dim_Z), Quaternion(0,0,0,1))
        points, lines = init_graph_markers(nodes, edges, nodes_id=2, edges_id=3, pose=pose, nodes_color=color, edges_color=color, linewidth=0.005, is_path=True)

        self.marker_pub.publish(points)
        self.marker_pub.publish(lines)

        if show_object:
            self.visualize_object(traj_msg.poses)
    
    # visualize object path with markers
    # TODO: we should use the existing marker to copy the actual shape instead of using hard coded values
    def visualize_object(self, poses, step_size=10):
        markers = MarkerArray()
        for step, pose in list(enumerate(poses))[0::step_size]:
                pose.position.z = 0.5*dim_Z
                progress = float(step) / len(poses)
                color = ColorRGBA(progress,0.0,1-progress,0.25)
                scale = Vector3(dim_X, dim_Y, dim_Z)
                markers.markers.append(init_marker(step, Marker.CUBE, scale=scale, pose=pose, color=color))
    
        self.marker_array_pub.publish(markers)
    
    # visualize whole planner data as graph
    def visualize_graph(self, graph_msg):
        #visualize_graph(graph_msg.nodes, graph_msg.edges)
        pose = Pose(Point(0,0,0.5*dim_Z), Quaternion(0,0,0,1))
    
        points, lines = init_graph_markers(graph_msg.nodes, graph_msg.edges, pose=pose)
    
        self.marker_pub.publish(points)
        self.marker_pub.publish(lines)
    
    
    def remove_all_markers(self):
        # clear visualized markers
        delete_all = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(delete_all)
        self.marker_array_pub.publish(MarkerArray(markers=[delete_all]))
