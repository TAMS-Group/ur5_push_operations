#!/usr/bin/env python

from copy import copy

import rospy
import tf
from tf import TransformListener
import actionlib
from lib import content_helper as ch

from tams_ur5_push_msgs.srv import ExecutePush, ExecutePushRequest, ExecutePushResponse
from tams_ur5_push_msgs.msg import PlanPushAction, PlanPushGoal
from tams_ur5_push_msgs.msg import MoveObjectAction, MoveObjectGoal, MoveObjectResult
from plan_visualization import PlanVisualization


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

import numpy as np
from math import pi


dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112

START = "start pose"
GOAL= "goal pose"

object_frame = "pushable_object_0"

def linalg_dist(p1, p2):
    return np.linalg.norm([p1.position.x - p2.position.x, p1.position.y - p2.position.y])

def se2Distance(p1, p2):
    p_dist = linalg_dist(p1, p2)
    o_dist = abs(ch.get_yaw(p1) - ch.get_yaw(p2)) % (2 * pi)
    o_dist = o_dist if o_dist < pi else 2 * pi - o_dist
    return p_dist + 0.5 * o_dist

def get_object_marker():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.header.frame_id = "/table_top"
    marker.scale = Vector3(dim_X, dim_Y, dim_Z)
    marker.pose.position.z = 0.5 * dim_Z
    return copy(marker)


def get_object_pose(object_frame, reference_frame):
    pose = None
    tf_l = TransformListener()
    try:
        print object_frame, reference_frame
        tf_l.waitForTransform(object_frame, reference_frame, rospy.Time(0), rospy.Duration(5.0))
        if (tf_l.frameExists(object_frame) and tf_l.frameExists(reference_frame)):
            p,q = tf_l.lookupTransform(reference_frame, object_frame, rospy.Time(0))
            pose = Pose(Point(*p), Quaternion(*q))
    except (tf.LookupException, tf.ConnectivityException):
        print "Unable to retrieve object pose!"
    del tf_l # kill listener to prevent annoying warnings
    return pose


# interactive marker controls
class InteractiveControls:

    def __init__(self,
                 object_marker,
                 interactive_start_state,
                 server= None,
                 menu_handler= None,
                 on_move_callback= None,
                 reference_frame = "table_top",
                 object_frame = "pushable_object_0",
                 start_pose= Pose(Point(-0.2, -0.2, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 goal_pose= Pose(Point(0.2, 0.2, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 start_color= ColorRGBA(0.0,0.0,1.0,1.0),
                 goal_color= ColorRGBA(1.0,0.0,0.0,1.0)):


        self.server = InteractiveMarkerServer("push_object_controls") if server is None else server
        self.object_marker = object_marker
        self.reference_frame = reference_frame
        self.object_frame = object_frame
        self.interactive_start_state = interactive_start_state
        self.poses = { START: start_pose, GOAL: goal_pose }
        self.colors = { START: start_color, GOAL: goal_color }

        self.menu_handler = menu_handler
        self.on_move_callback = on_move_callback

        # set goal pose to object pose on start if start pose is not interactive
        if not self.interactive_start_state:
            pose = get_object_pose(object_frame, reference_frame)
            if pose is not None:
                self.poses[GOAL] = pose
            else:
                print "Error! Object pose could not be found"

    def set_menu_handler(self, menu_handler):
        self.menu_handler = menu_handler

    def set_on_move_callback(self, callback):
        self.on_move_callback = callback

    def create_interactive_marker(self, name, active = True):
        # retrieve pushable object marker
        marker = copy(self.object_marker)
        marker.header.frame_id = ""
        marker.color = self.colors[name]
        marker.color.a = 1.0 if active else 0.1

        # create interactive marker
        int_marker = InteractiveMarker()
        int_marker.name = name
        int_marker.description = name
        int_marker.header.frame_id = self.reference_frame
        int_marker.scale = 0.5
        int_marker.pose = self.poses[name]
        int_marker.pose.position.z = marker.pose.position.z
        marker.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))

        # Create a controllable marker
        control = InteractiveMarkerControl( always_visible=True )
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation = Quaternion(0,1,0,1)
        control.markers.append( marker )
        int_marker.controls.append( control )

        # if active show interaction arrows
        if active:

            control = InteractiveMarkerControl( name="rotate_z" )
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            control.orientation=Quaternion(0,1,0,1)
            int_marker.controls.append(control)

            control = InteractiveMarkerControl( name="move_x" )
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation = Quaternion(0,0,1,1)
            int_marker.controls.append(control)

            control = InteractiveMarkerControl( name="move_y" )
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation = Quaternion(1,0,0,1)
            int_marker.controls.append(control)

        # register marker and move event callback
        self.server.insert(int_marker, self.on_move_cb)

        # apply context menu to marker
        if self.menu_handler is not None:
            self.menu_handler.apply( self.server, int_marker.name )

    def get_start_pose(self):
        if not self.interactive_start_state:
            pose = get_object_pose(self.object_frame, self.reference_frame)
            if pose is not None:
                self.poses[START] = pose
        return self.poses[START]

    def get_goal_pose(self):
        return self.poses[GOAL]

    def reset_markers(self, active=True):
        if self.interactive_start_state:
            self.create_interactive_marker( START, active)

        self.create_interactive_marker( GOAL, active)
        self.server.applyChanges()


    def on_move_cb(self, feedback):
        if feedback.event_type == feedback.MOUSE_UP:
            name = feedback.marker_name
            if name in self.poses:
                if self.interactive_start_state or name == GOAL:
                    self.poses[name] = feedback.pose

        if self.on_move_callback is not None:
            self.on_move_callback(feedback)


# Menu and interaction event handler

class EventHandler:

    def __init__(self, controls, plan_viz):
        self.highlight_solution = False
        self.plan_viz = plan_viz

        self.menu_handler = MenuHandler()
        self.menu_handler.insert( "Plan", callback=self.onPlan )
        self.menu_handler.insert( "Reset", callback=self.onReset )
        self.menu_handler.insert( "Execute", callback=self.onExecute )
        self.menu_handler.insert( "Run MPC", callback=self.onRunMPC )
        self.menu_handler.insert( "Run multi-step MPC", callback=self.onRunMultiStepMPC )

        self.controls = controls
        self.controls.set_menu_handler(self.menu_handler)
        self.controls.set_on_move_callback( self.onMove )
        self.controls.reset_markers(True)

        self.planner_client = actionlib.SimpleActionClient('/push_plan_action', PlanPushAction)

        rospy.wait_for_service("push_execution")
        self.execute_service = rospy.ServiceProxy("push_execution", ExecutePush)


    def onMove(self, feedback ):
        if feedback.event_type == feedback.MOUSE_DOWN:
            # if solution is highlighted and markers are hidden - show them again
            if self.highlight_solution:
                self.plan_viz.remove_all_markers()
                self.highlight_solution = False
                self.controls.reset_markers(True)

    def onPlan( self, feedback ):
        print "plan"
        # get start and goal poses
        start_pose = self.controls.get_start_pose()
        goal_pose = self.controls.get_goal_pose()

        # call action
        self.solution = self.call_push_plan_action("object_id", start_pose, goal_pose)

        # hide markers
        self.highlight_solution = True
        self.controls.reset_markers(False)

    def onReset( self, feedback ):
        print "reset"
        self.highlight_solution = False
        self.controls.reset_markers(True)

    def onExecute( self, feedback ):
        print "execute"
        rospy.wait_for_service("push_execution")
        print "found service"
        execute_push = rospy.ServiceProxy("push_execution", ExecutePush)
        for push in self.solution.trajectory.pushes:
            push.approach.frame_id = object_frame
            print "execute push", push
            try:
                resp = execute_push(push)
                if not resp.result:
                    break
            except rospy.ServiceException as e:
                print("Service did not process request: " + str(e))
                break

    def onRunMPC( self, feedback ):
        pose = self.controls.get_start_pose()
        goal = self.controls.get_goal_pose()
        attempts = 0

        # run MPC as long as goal is not reached and attempts
        while se2Distance(pose, goal) > 0.05 and attempts < 10:

            # call push plan action
            plan = self.call_push_plan_action("object_id", pose, goal)

            # highlight plan
            self.highlight_solution = True
            self.controls.reset_markers(False)

            # attempt to execute the push
            push = plan.trajectory.pushes[0]
            attempts += 1

            if self.execute_push(push):
                attempts = 0 # reset attempts

            pose = self.controls.get_start_pose()


    def onRunMultiStepMPC( self, feedback ):
        pose = self.controls.get_start_pose()
        goal = self.controls.get_goal_pose()
        replan = True
        attempts = 0
        step = 0
        push = None

        # run MPC as long as goal is not reached and attempts
        while se2Distance(pose, goal) > 0.05 and attempts < 10 :

            # replan, if necessary
            if replan :

                # call push plan action
                plan = self.call_push_plan_action("object_id", pose, goal, push)
                step = 0

                # highlight plan
                self.highlight_solution = True
                self.controls.reset_markers(False)

            # replan, if end of trajectory is reached
            if not step < len(plan.trajectory.pushes):
                replan = True
                continue

            # attempt to execute the push
            push = plan.trajectory.pushes[step]
            attempts += 1

            if self.execute_push(push):
                attempts = 0 # reset attempts
                step += 1 # increase trajectory step
            else:
                replan = True

            # query new object pose
            pose = self.controls.get_start_pose()
            # replan, if the object deviates from path
            replan = se2Distance(pose, plan.trajectory.poses[step]) > 0.05


    def execute_push(self, push):
        print "execute push", push
        try:
            result = self.execute_service(push)
            return result.result
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))
        return False

    def call_push_plan_action(self, object_id, start_pose, goal_pose, push=None):

        #create goal
        goal = PlanPushGoal()
        goal.object_id = object_id
        goal.start_pose = start_pose
        goal.goal_pose = goal_pose
        if push is not None:
          goal.last_push = push

        print start_pose, goal_pose

        # call client
        self.planner_client.wait_for_server()
        self.planner_client.send_goal(goal)
        self.planner_client.wait_for_result()

        # receive result
        result = self.planner_client.get_result()

        self.plan_viz.visualize_trajectory(result.trajectory)
        self.plan_viz.visualize_graph(result.planner_data)
        return result


if __name__=="__main__":
    rospy.init_node("interactive_push_markers_node")

    interactive_start_state = rospy.get_param('~interactive_start_state', False)

    # prepare plan visualization with object marker
    marker = get_object_marker()
    plan_viz = PlanVisualization(marker, marker.pose.position.z);

    # prepare interactive marker controls
    controls = InteractiveControls(marker, interactive_start_state)

    # handle menu events
    event_handler = EventHandler(controls, plan_viz)

    rospy.spin()
