#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA

menu_handler = MenuHandler()

dim_X = 0.162
dim_Y = 0.23
dim_Z = 0.112

#def processFeedback( feedback ):
#    s = "Feedback from marker '" + feedback.marker_name
#    s += "' / control '" + feedback.control_name + "'"
#
#    mp = ""
#    if feedback.mouse_point_valid:
#        mp = " at " + str(feedback.mouse_point.x)
#        mp += ", " + str(feedback.mouse_point.y)
#        mp += ", " + str(feedback.mouse_point.z)
#        mp += " in frame " + feedback.header.frame_id
#
#    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
#        rospy.loginfo( s + ": button click" + mp + "." )
#    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
#        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
#    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
#        rospy.loginfo( s + ": pose changed")
def processFeedback( feedback ):
    print feedback

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

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)

def makePlaneMarker( marker, name, pose, show_controls = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "table_top"
    int_marker.pose = pose
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
    
    if show_controls: 

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

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


if __name__=="__main__":
    rospy.init_node("interactive_push_markers_node")

    server = InteractiveMarkerServer("push_object_controls")

    #menu_handler.insert( "First Entry", callback=processFeedback )
    #menu_handler.insert( "Second Entry", callback=processFeedback )
    #sub_menu_handle = menu_handler.insert( "Submenu" )
    #menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    #menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )
    
    sp = Pose()
    sp.orientation.w = 1.0
    sp.position.y = -0.2
    sp.position.x = -0.2

    gp = Pose()
    gp.orientation.w = 1.0
    gp.position.x = 0.2
    gp.position.y = 0.2
    start_box = getBox(dim_X, dim_Y, dim_Z)

    makePlaneMarker( start_box, "start pose", sp, True)

    goal_box = getBox(dim_X, dim_Y, dim_Z, ColorRGBA(1.0,0.0,0.0,1.0) )
    makePlaneMarker( goal_box, "goal pose", gp, True)

    server.applyChanges()

    rospy.spin()
