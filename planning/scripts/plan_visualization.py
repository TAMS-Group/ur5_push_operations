#!/usr/bin/env python

import rospy
from tams_ur5_push_msgs.msg import PushTrajectory 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import *
from graph_msgs.msg import GeometryGraph, Edges
from copy import copy

from lib.marker_helper import init_marker, init_graph_markers

class PlanVisualization:

    def __init__(self, object_marker, v_offset=0.0):
        #init publishers
        self.marker_array_pub = rospy.Publisher("/push_trajectory_markers", MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=1)

        self.object_marker = object_marker
        self.object_marker.action = Marker.ADD
        self.v_offset = v_offset

    # visualize trajectory as graph
    def visualize_trajectory(self, traj_msg, show_object=True):
        nodes = [p.position for p in traj_msg.poses]
        edges = [Edges([i, i+1], [1.0, 1.0]) for i in range(len(nodes)-1)]
        edges.append(Edges()) # add empty Edges() for last node
        color = ColorRGBA(0.0,1.0,0.0,1.0)
        pose = Pose(Point(0,0,self.v_offset), Quaternion(0,0,0,1))
        points, lines = init_graph_markers(nodes, edges, nodes_id=2, edges_id=3, pose=pose, nodes_color=color, edges_color=color, linewidth=0.005, is_path=True)

        self.marker_pub.publish(points)
        self.marker_pub.publish(lines)

        if show_object:
            self.visualize_object(traj_msg.poses)
    
    # visualize object path with markers
    def visualize_object(self, poses, step_size=10):
        markers = MarkerArray()
        for step, pose in list(enumerate(poses))[0::step_size]:
            marker = copy(self.object_marker)
            marker.id = step
            marker.pose = pose
            marker.pose.position.z = self.v_offset
            progress = float(step) / len(poses)
            marker.color = ColorRGBA(progress, 0.0, 1-progress, 0.25)
            markers.markers.append(marker)
    
        self.marker_array_pub.publish(markers)
    
    # visualize whole planner data as graph
    def visualize_graph(self, graph_msg):
        #visualize_graph(graph_msg.nodes, graph_msg.edges)
        pose = Pose(Point(0,0,self.v_offset), Quaternion(0,0,0,1))

        points, lines = init_graph_markers(graph_msg.nodes, graph_msg.edges, pose=pose)

        self.marker_pub.publish(points)
        self.marker_pub.publish(lines)
    
    
    # clear visualized markers
    def remove_all_markers(self):
        self.marker_pub.publish(Marker(action=Marker.DELETEALL))
        self.marker_array_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
