#!/usr/bin/env python

import rospy
from tams_ur5_push_execution.msg import PushTrajectory 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from graph_msgs.msg import GeometryGraph, Edges

dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112

def visualize_push_trajectory(traj_msg):
    global marker_pub
    markers = MarkerArray()

    # visualize as graph
    nodes = [p.position for p in traj_msg.poses]
    edges = [Edges([i, i+1], [1.0, 1.0]) for i in range(len(nodes)-1)]
    edges.append(Edges())
    color = ColorRGBA(0.0,1.0,0.0,1.0)
    visualize_graph(nodes, edges, nodes_id=2, edges_id=3, nodes_color=color, edges_color=color, linewidth=0.003, is_path=True)

    # show box path
    for i,pose in enumerate(traj_msg.poses):
        marker = Marker()
        marker.header.frame_id = "/table_top"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = dim_X
        marker.scale.y = dim_Y
        marker.scale.z = dim_Z
        marker.color.a = 0.1
        progress = float(i) / len(traj_msg.poses)
        marker.color.b = 1 - progress
        marker.color.r = progress
        marker.id = i
        marker.pose = pose
        marker.pose.position.z = 0.5*dim_Z
        markers.markers.append(marker)
    marker_pub.publish(markers)



#def visualize_planner_data(graph_msg):
#    global data_pub
#
#    pose = Pose()
#    pose.position.z = 0.5 * dim_Z
#    pose.orientation.w = 1.0
#
#    points = getMarker(0, Marker.POINTS, scale=Vector3(0.0025, 0.0025, 0.0), pose=pose)
#    lines = getMarker(1, Marker.LINE_LIST, scale=Vector3(0.001,0,0), pose=pose)
#
#    for i,(point, adjacent) in enumerate(zip(graph_msg.nodes, graph_msg.edges)):
#        points.points.append(point)
#        points.colors.append(ColorRGBA(1.0,0.0,0.0,1.0))
#        for n in adjacent.node_ids:
#            lines.points.append(point)
#            lines.points.append(graph_msg.nodes[n])
#        
#    data_pub.publish(points)
#    data_pub.publish(lines)

def visualize_planner_data(graph_msg):
    visualize_graph(graph_msg.nodes, graph_msg.edges)

def visualize_graph(nodes, edges, nodes_id=0, edges_id=1, 
        nodes_color=ColorRGBA(0.0,0.0,0.0,1.0),
        edges_color=ColorRGBA(0.0,0.0,0.0,0.5),
        linewidth=0.001,
        is_path=False):
    global graph_pub

    pose = Pose()
    pose.position.z = 0.5 * dim_Z
    pose.orientation.w = 1.0

    point_width = 0.005 if is_path else 0.002
    points = getMarker(nodes_id, Marker.POINTS, scale=Vector3(point_width, point_width, 0.0), pose=pose)
    lines = getMarker(edges_id, Marker.LINE_LIST, scale=Vector3(linewidth,0,0), pose=pose, color=edges_color)

    for i,(point, adjacent) in enumerate(zip(nodes, edges)):
        points.points.append(point)
        color = nodes_color

        # change color if path is shown
        if is_path:
            if i == 0:
                color = ColorRGBA(0.0,0.0,1.0,1.0)
            elif i == len(nodes) - 1:
                color = ColorRGBA(1.0,0.0,0.0,1.0)

        points.colors.append(color)
        for n in adjacent.node_ids:
            lines.points.append(point)
            lines.points.append(nodes[n])
        
    data_pub.publish(points)
    data_pub.publish(lines)



# attempt to visualize weights - however all weights are set to 1.0
#def visualize_planner_data(graph_msg):
#    global data_pub
#
#    pose = Pose()
#    pose.position.z = 0.5 * dim_Z
#    pose.orientation.w = 1.0
#
#    id_count = 0
#    points = getMarker(id_count, Marker.POINTS, scale=Vector3(0.0025, 0.0025, 0.0), pose=pose)
#
#    for i,(point, adjacent) in enumerate(zip(graph_msg.nodes, graph_msg.edges)):
#        points.points.append(point)
#        points.colors.append(ColorRGBA(1.0,0.0,0.0,1.0))
#        for n, weight in zip(adjacent.node_ids, adjacent.weights):
#            id_count += 1
#            if weight < 1.0:
#                print weight
#            lines = getMarker(id_count, Marker.LINE_LIST, scale=Vector3(0.001 * weight,0,0), pose=pose)
#            lines.points.append(point)
#            lines.points.append(graph_msg.nodes[n])
#            data_pub.publish(lines)
#        
#    data_pub.publish(points)

def getMarker(marker_id, marker_type, pose=None, scale=None, color=None):
    marker = Marker()
    marker.header.frame_id = "/table_top"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0

    if scale is not None:
        marker.scale = scale
    if pose is not None:
        marker.pose = pose
    if color is not None:
        marker.color = color
    return marker

def visualization_node():
    global marker_pub, data_pub
    rospy.init_node('push_trajectory_visualization_node');

    traj_sub = rospy.Subscriber('/push_trajectory', PushTrajectory, visualize_push_trajectory, queue_size=1)
    data_sub = rospy.Subscriber('/push_planner_graph', GeometryGraph, visualize_planner_data, queue_size=1)

    marker_pub = rospy.Publisher("/push_trajectory_markers", MarkerArray, queue_size=1)
    data_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=1)
    graph_pub = rospy.Publisher("/push_planner_graph_markers", Marker, queue_size=10)


    print "Ready to visualize push trajectories"
    rospy.spin()

if __name__=="__main__":
    print "init predictive sampler node"
    visualization_node()
