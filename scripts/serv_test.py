#!/usr/bin/env python

from ahmedrrt.srv import makePlan,makePlanResponse
import rospy
from geometry_msgs.msg import Point


def handler(msg):
    print msg.pose
    p0=Point()
    p0.x=p0.y=0.0
    p1=Point()
    p1.x=p1.y=1.0
    path=[p0,p1]
    return makePlanResponse(path)





def serverNode():
    rospy.init_node('pathPlannerServer')
    s = rospy.Service('makePlane', makePlan, handler)
    print "Ready to add two ints."
    rospy.spin()




if __name__ == "__main__":
    serverNode()
