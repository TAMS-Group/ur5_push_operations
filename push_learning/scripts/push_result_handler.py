#!/usr/bin/env python
import rospy
from collections import deque
from tams_ur5_push_execution.msg import ExplorePushesActionFeedback, ExplorePushesActionResult
from visualization_msgs.msg import Marker

has_object_representation=False
epoch_finished=False

result_queue=deque([])
obj_dim=None

def marker_callback(marker):
    global marker_sub, has_object_representation, obj_dim
    if(not has_object_representation and len(result_queue) > 0 and marker.type == 1 and
            result_queue[0].push.approach.frame_id == marker.header.frame_id):
        rospy.loginfo("received object information")
        has_object_representation = True
        obj_dim = marker.scale
        marker_sub.unregister()

def push_feedback_callback(feedback_msg):
    global result_queue
    result_queue.append(feedback_msg.feedback)

def push_result_callback(result):
    global push_sub
    epoch_finished=True
    push_sub.unregister()
    rospy.loginfo("Finished push exploration sucessfully with %s attemtps after %s seconds", result.result.attempts, result.result.elapsed_time)

def handle_push_feedback(feedback):
    rospy.loginfo("Handling push feedback ")

def listener():
    global marker_sub, push_sub, result_queue, has_object_representation, epoch_finished
    rospy.init_node('push_result_listener', anonymous=True)
    marker_sub = rospy.Subscriber("/pushable_objects", Marker, marker_callback)
    push_sub = rospy.Subscriber("/explore_pushes_action/feedback", ExplorePushesActionFeedback, push_feedback_callback)
    push_result_sub = rospy.Subscriber("/explore_pushes_action/result", ExplorePushesActionResult, push_result_callback)
    rate = rospy.Rate(2)
    while(not rospy.is_shutdown() and not epoch_finished):
        if(has_object_representation and len(result_queue)>0):
            handle_push_feedback(result_queue.popleft())
        rate.sleep()


if __name__ == '__main__':
    listener()
