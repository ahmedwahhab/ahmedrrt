#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import actionlib_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from ahmedrrt.srv import makePlan as makeplan
from ahmedrrt.srv import makePlanResponse


from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil,arctan2,append
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from numpy import cos,sin
from geometry_msgs.msg import Twist

from copy import deepcopy
from numpy import arange,dot,linalg,power
from math import sqrt
#-----------------------------------------------------


tfLisn=tf.TransformListener()
dt=0
robotSpeed=0




def getPosition():
	global tfLisn
	cond=0;	
	while cond==0:	
		try:
			(trans,rot) = tfLisn.lookupTransform('/map', '/base_link', rospy.Time(0))
			cond=1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			cond==0
	
	(roll,pitch,yaw)=tf.transformations.euler_from_quaternion(rot)  # quaternion to roll,pitch,yaw conversion
	return [trans[0],trans[1]],yaw
	
def getTrajectory(waypoints,robotSpeed,dt):
	t_initial=0.0
	BIGPOLYNIMOALX = []
	BIGPOLYNIMOALY = []
	BIGTIME = 0.0

	vx_initial = 0.0 #m/s
	vy_initial = 0.0 #m/s

	t_initial = 0.0 #sec
	for W in range(0,len(waypoints)-1):

		x_initial = waypoints[W].x
		x_final = waypoints[W+1].x
		y_initial =waypoints[W].y
		y_final = waypoints[W+1].y
	
		#Velocity at END = 0 m/s:
		if W==len(waypoints)-1:
			vx_final = 0.0 #m/s
			vy_final = 0.0 #m/s
		#Velocity at WayPoints = 0.01 m/s:
		else:
			if (x_initial<x_final):
				#vx_final = 0.01 #m/s	#for square-like shapes
				vx_final = 0.0 #m/s	#for continuous shapes (rose - circle..)
			else: 
				#vx_final = -0.01 #m/s	#for square-like shapes
				vx_final = -0.0 #m/s	#for continuous shapes (rose - circle..)
			
			if (y_initial<y_final):	
				#vy_final = 0.01 #m/s	#for square-like shapes
				vy_final = 0.0 #m/s	#for continuous shapes (rose - circle..)
			else:
				#vy_final = -0.01 #m/s	#for square-like shapes
				vy_final = -0.0 #m/s	#for continuous shapes (rose - circle..)


		t_final = (1.0/robotSpeed) * sqrt( (x_final-x_initial)*(x_final-x_initial) + (y_final-y_initial)*(y_final-y_initial) ) #sec = 10sec per meter
	
		#Quadratic Polynomial Coefficients Matrix:
		print t_initial, '      ' , t_final
		
		Polymatrix =	[[1.0, t_initial,	t_initial*t_initial,	t_initial*t_initial*t_initial	],
				[0.0, 1.0,		2.0*t_initial,		3.0*t_initial*t_initial		],
				[1.0, t_final,		t_final*t_final,	t_final*t_final*t_final		],
				[0.0, 1.0,		2.0*t_final,		3.0*t_final*t_final		]]

		Cx = [[x_initial],[vx_initial],[x_final],[vx_final]]	  
		Cy = [[y_initial],[vy_initial],[y_final],[vy_final]]
		
		try:   
			Polyinvmatrix = linalg.inv(Polymatrix) #matrix inverse
		except:
			return None
		Ax = dot(Polyinvmatrix,Cx) #matrix multiplication to obtain X polynomial coefiicients
		Ay = dot(Polyinvmatrix,Cy) #matrix multiplication to obtain Y polynomial coefiicients

		#Polynomials Definitions:
		tt=arange(t_initial,t_final,dt) 	
		PolynomialX = Ax[0] + Ax[1]*tt + Ax[2]*power(tt,2) + Ax[3]*power(tt,3)
		PolynomialY = Ay[0] + Ay[1]*tt + Ay[2]*power(tt,2) + Ay[3]*power(tt,3)
	
		#Concatenations of segments:
		for el in PolynomialX:
			BIGPOLYNIMOALX.append(el)
		for els in PolynomialY:
			BIGPOLYNIMOALY.append(els)
	
		BIGTIME+=t_final #Addition of current segment's time to the overall BIGTIME
	
		vx_initial = deepcopy(vx_final) #Next WayPoint's initial velocity is the previous WayPoint's final velocity
		vy_initial = deepcopy(vy_final)

	#### <<< END of FULL PATH LOOP >>>

	t=arange(t_initial,BIGTIME,dt) #time used by the controller
	
	return BIGPOLYNIMOALX,BIGPOLYNIMOALY,t





def trajectoryControl(pathx,pathy,kx,ky,V_MAX,Omega_MAX,b):
	global pub,rate
	vel=Twist()
	
	for i in range(len(pathx)):
		x_des=[pathx[i],pathy[i]]
		x_robot,yaw=getPosition()
		xb=x_robot[0]+b*cos(yaw)  #  (yaw is theta around z)
		yb=x_robot[1]+b*sin(yaw)
	
		ex=x_des[0]-xb    # error vectors ex and ey
		ey=x_des[1]-yb    
	
	
		Xb_dot = kx*ex
		Yb_dot = ky*ey
	
	
		V = cos(yaw)*Xb_dot + sin(yaw)*Yb_dot              # Velocity calculation
	
		#Saturation for V:
		if (V > V_MAX):
			V = V_MAX

		
		Omega = (cos(yaw)*Yb_dot - sin(yaw)*Xb_dot)/b  # Omega calculation
	
		#Saturation for Omega:
		if (Omega > Omega_MAX):	
			Omega = Omega_MAX
		
	
		vel.linear.x = V         # feeding velocity control
		vel.angular.z = Omega    # feeding omega control 
	
		pub.publish(vel)       # Publishing feeded control
		rate.sleep()	
# Subscribers' callbacks------------------------------
def callBack(data):
	global dt,robotSpeed
	PathPlanner = rospy.ServiceProxy('makePlane', makeplan)
	print data.pose
	try:
		response = PathPlanner(data.pose)
	except:
		return None
	
	waypoints=response.pathPoints

	waypoints=waypoints[::-1]
	print waypoints
	#getTrajectory(waypoints,robotSpeed,dt)
	pathx,pathy,t=getTrajectory(waypoints,robotSpeed,dt)
	
	#trajectoryControl(pathx,pathy,kx,ky,V_MAX,Omega_MAX,b)
	trajectoryControl(pathx,pathy,1,1,0.3,1.0,0.2)
  
    



#Node
def node():
	global tfLisn,pub,dt,robotSpeed,rate
	rospy.init_node('move_base')
	robotSpeed=0.5
	dt=0.05
	rate=rospy.Rate(5/robotSpeed)
	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	
	condit=0
	while condit==0:
		try:
			tfLisn.waitForTransform('/map', '/base_link', rospy.Time(0),rospy.Duration(10.0))
			condit=1
		except:
			pass

	
	try:
		rospy.wait_for_service('/makePlane')
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, callBack)
		rospy.spin()
	except:
		rospy.logwarn("cannot start planner client")
		
	


	

	
#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 




    
    
    
    
