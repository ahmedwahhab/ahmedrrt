import random,math
from geometry_msgs.msg import Point
from copy import copy
from numpy import linalg as LA
from numpy import array,floor,ceil,arctan2,cos,sin

def steer(x0,x1,eta):
	r=LA.norm(x0-x1)
	if r>eta:
		r=eta
	
	theta=arctan2(x1[1]-x0[1],x1[0]-x0[0])

	a=x0[0]+r*cos(theta)
	b=x0[1]+r*sin(theta)
	xnew=array([a,b])
	return xnew
		
def gridCheck(mapData,Xp):
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y

	width=mapData.info.width
	Data=mapData.data
	# check if points are in freespace or not
	# c=100 means grid cell occupied
	# c=0 means grid cell is free
	#c=-1 means grid cell is unknown
	index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) )
	c=100
	if int(index) < len(Data):
		c=Data[int(index)]

	return c

	# ObstacleFree function-------------------------------------

def LineCollisionCheck(first,second,mapsub):
	xnearest=array(first[0:2])
	xnew=array(second[0:2])
	out=0
	rez=mapsub.info.resolution*.2
	stepz=int(ceil(LA.norm(xnew-xnearest)/rez))
	xi=xnearest
	obs=0
	unk=0
	 

	for c in range(0,stepz):
		xi=steer(xi,xnew,rez)
		if (gridCheck(mapsub,xi) >5):
			obs=1
			break
		elif (gridCheck(mapsub,xi) ==-1):
			unk=1
			break


	if (unk==1):
		out=True

	if (obs==1):
		out=False

	if (obs!=1 and unk!=1):
		out=True


	return out



def GetPathLength(path):
    le = 0
    for i in range(0,len(path) - 1):
        dx = path[i + 1].x - path[i].x
        dy = path[i + 1].y - path[i].y
        d = math.sqrt(dx * dx + dy * dy)

        le += d

    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1].x - path[i].x
        dy = path[i + 1].y - path[i].y
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen


    x = path[ti].x + (path[ti + 1].x - path[ti].x) * partRatio
    y = path[ti].y + (path[ti + 1].y - path[ti].y) * partRatio


    return [x, y, ti]




def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")
    le = GetPathLength(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = GetTargetPoint(path, pickPoints[0])
        second = GetTargetPoint(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
        	continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        p=Point()
        p.x=first[0]
        p.y=first[1]
        
        newPath.append(copy(p))
        p.x=second[0]
        p.y=second[1]
        
        newPath.append(copy(p))
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)

    return path
