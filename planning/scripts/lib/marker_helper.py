from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA

def init_marker(marker_id, marker_type, pose=None, scale=None, color=None, frame_id="/table_top"):
    marker = Marker()
    marker.header.frame_id = frame_id
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

def init_graph_markers(nodes, edges, nodes_id=0, edges_id=1, 
        pose=None,
        nodes_color=ColorRGBA(0.0,0.0,0.0,1.0),
        edges_color=ColorRGBA(0.0,0.0,0.0,0.5),
        linewidth=0.001,
        is_path=False):
    global graph_pub

    if pose is None:
        pose = Pose()
        pose.orientation.w = 1.0

    point_width = 0.005 if is_path else 0.002
    points = init_marker(nodes_id, Marker.POINTS, scale=Vector3(point_width, point_width, 0.0), pose=pose)
    lines = init_marker(edges_id, Marker.LINE_LIST, scale=Vector3(linewidth,0,0), pose=pose, color=edges_color)

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

    return points, lines
