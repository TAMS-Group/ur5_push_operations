#!/usr/bin/env python

import copy

import rospy
import tf
from tf import TransformListener
import actionlib

from push_planning.msg import PushPlanAction, PushPlanGoal
import push_planner_visualization as ppv


from tams_ur5_push_execution.srv import ExecutePush, ExecutePushRequest, ExecutePushResponse
from tams_ur5_push_execution.msg import MoveObjectAction, MoveObjectGoal, MoveObjectResult


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA


menu_handler = MenuHandler()
server = None

dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112

reference_frame = "table_top"
object_frame = "pushable_object_0"

START = "start pose"
GOAL= "goal pose"

poses = { START: Pose(Point(-0.2, -0.2, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
          GOAL: Pose(Point(0.2, 0.2, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)) }

colors = { START: ColorRGBA(0.0,0.0,1.0,1.0),
           GOAL: ColorRGBA(1.0,0.0,0.0,1.0) }


# marker/control instatniation

def getBox( dimX, dimY, dimZ, color=ColorRGBA(0.0,0.0,1.0,1.0) ):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = dimX
    marker.scale.y = dimY
    marker.scale.z = dimZ
    marker.color = color
    return marker

def makeBoxControl( msg, marker ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( marker )
    msg.controls.append( control )
    return control

def makePlanarIntMarker( name, active = True):
    marker = getBox(dim_X, dim_Y, dim_Z, colors[name])
    marker.color.a = 1.0 if active else 0.1
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = reference_frame
    int_marker.pose = poses[name]
    int_marker.pose.position.z = marker.scale.z * .5
    int_marker.scale = 0.5

    int_marker.name = name
    int_marker.description = name

    # insert a box
    makeBoxControl(int_marker, marker)
    int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls[0].orientation.w = 1
    int_marker.controls[0].orientation.x = 0
    int_marker.controls[0].orientation.y = 1
    int_marker.controls[0].orientation.z = 0
    
    if active: 

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)


        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)


    server.insert(int_marker, onMove)
    menu_handler.apply( server, int_marker.name )

def call_push_plan_action(object_id, start_pose, goal_pose):
    global planner_client, last_solution

    #create goal
    goal = PushPlanGoal()
    goal.object_id = object_id
    goal.start_pose = start_pose
    goal.goal_pose = goal_pose

    print start_pose, goal_pose

    # call client
    planner_client.wait_for_server()
    planner_client.send_goal(goal)
    planner_client.wait_for_result()

    # receive result
    result = planner_client.get_result()
    last_solution = result

    ppv.visualize_object_trajectory(result.trajectory)
    ppv.visualize_planner_data(result.planner_data)


def reset_markers(active=True):
    global start_pose_is_interactive
    if start_pose_is_interactive:
        makePlanarIntMarker( START, active)

    makePlanarIntMarker( GOAL, active)
    server.applyChanges()

def get_object_pose(object_frame, reference_frame):
    pose = None
    tf_l = TransformListener()
    try:
        print object_frame, reference_frame
        tf_l.waitForTransform(object_frame, reference_frame, rospy.Time(0), rospy.Duration(5.0))
        if (tf_l.frameExists(object_frame) and tf_l.frameExists(reference_frame)):
            p,q = tf_l.lookupTransform(object_frame, reference_frame, rospy.Time(0))
            pose = Pose(Point(*p), Quaternion(*q))
    except (tf.LookupException, tf.ConnectivityException):
        print "Unable to retrieve object pose!"

    return pose


# Callbacks

def onMove( feedback ):
    global highlight_solution, start_pose_is_interactive
    event = feedback.event_type
    name = feedback.marker_name

    if event == feedback.MOUSE_UP:
        if(name == GOAL or ( start_pose_is_interactive and name == START) ):
            poses[name] = feedback.pose

    if event == feedback.MOUSE_DOWN:
        # if solution is highlighted and markers are hidden - show them again
        if highlight_solution:
            ppv.remove_all_markers()
            highlight_solution = False
            reset_markers(True)


def onPlan( feedback ):
    global highlight_solution, start_pose_is_interactive
    print "plan"

    # set start pose
    start_pose = poses[START]
    if not start_pose_is_interactive:
        pose = get_object_pose(object_frame, reference_frame)
        if pose is not None:
            start_pose = pose

    # call action
    call_push_plan_action("object_id", start_pose, poses[GOAL])

    # hide markers
    highlight_solution = True
    reset_markers(False)


def onReset( feedback ):
    global highlight_solution
    print "reset"
    highlight_solution = False
    reset_markers(True)


#def onExecute( feedback ):
#    global last_solution
#    print "execute"
#    rospy.wait_for_service("push_execution")
#    print "found service"
#    execute_push = rospy.ServiceProxy("push_execution", ExecutePush)
#    for push in last_solution.trajectory.pushes:
#        print "execute push", push
#        resp = execute_push(push)
#        if not resp.result:
#            break

def onExecute( feedback ):
    global last_solution
    client = actionlib.SimpleActionClient("move_object_action", MoveObjectAction)
    client.wait_for_server()
    for pose in last_solution.trajectory.poses:
        goal = MoveObjectGoal(object_id=object_frame, target=pose)
        client.send_goal(goal)
        client.wait_for_result()

def init_interaction_server():
    global planner_client, highlight_solution, start_pose_is_interactive

    highlight_solution = False
    menu_handler.insert( "Plan", callback=onPlan )
    menu_handler.insert( "Reset", callback=onReset )
    menu_handler.insert( "Execute", callback=onExecute )

    start_pose_is_interactive = rospy.get_param('start_pose_is_interactive', False)

    # set goal pose to object pose on start
    if not start_pose_is_interactive:
        pose = get_object_pose(object_frame, reference_frame)
        if pose is not None:
            poses[GOAL] = pose
        else:
            print "Error! Object pose could not be found"

    reset_markers(True)

    ppv.init_publishers()

    planner_client = actionlib.SimpleActionClient('/push_plan_action', PushPlanAction)



if __name__=="__main__":
    rospy.init_node("interactive_push_markers_node")
    server = InteractiveMarkerServer("push_object_controls")
    init_interaction_server()
    rospy.spin()
