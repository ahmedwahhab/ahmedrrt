#!/usr/bin/env python


#--------Include modules---------------

import rospy

from nav_msgs.msg import OccupancyGrid

#-----------------------------------------------------

mapData=OccupancyGrid()




# Subscribers' callbacks------------------------------
def mapCallBack(data):
    global mapData
    mapData=data
    


#Node
def node():
	global mapData
	rospy.init_node('mapSub')
	rospy.Subscriber("costmap_node/costmap/costmap", OccupancyGrid, mapCallBack)
	rate=rospy.Rate(100)
	while not rospy.is_shutdown():
		print mapData.header.seq
		rate.sleep()
	
#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
