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

def _LineCollisionCheck(first,second,mapsub):
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
