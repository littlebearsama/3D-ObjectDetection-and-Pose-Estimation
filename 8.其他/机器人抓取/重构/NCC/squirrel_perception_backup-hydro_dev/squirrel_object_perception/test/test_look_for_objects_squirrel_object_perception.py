#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_object_perception')
import rospy

import actionlib
import yaml
from squirrel_object_perception_msgs.msg import *

test_squirrel_object_perception_goal = open("test/testdata_look_for_objects_squirrel_object_perception", 'r')

def squirrel_object_perception_client():
    client = actionlib.SimpleActionClient('/squirrel_object_perception', LookForObjectsAction)
    client.wait_for_server()
    
    goal =LookForObjectsGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_squirrel_object_perception_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('squirrel_object_perception_test')
    print squirrel_object_perception_client()
