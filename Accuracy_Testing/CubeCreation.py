import numpy as np
import math
import plotly.graph_objects as go


def sphericalRelative(t1_rad, t2_rad, targetP, r):    
    if t1_rad > 1.5707:  #for semisphere
        t1_rad = t1_rad/2
    
    # for reference https://mathinsight.org/spherical_coordinates
    x = targetP[0] + (r * math.sin(t1_rad) * math.cos(t2_rad))
    y = targetP[1] + (r * math.sin(t1_rad) * math.sin(t2_rad))
    z = targetP[2] + (r * math.cos(t1_rad))
    
    quadrant = 0
    
    if 0 >= t2_rad and t2_rad <= 1.5707963267948966:
        quadrant = 1
    elif 1.5882496193148399 >= t2_rad and t2_rad <= 3.141592653589793:
        quadrant = 2
    elif 3.1590459461097367 >= t2_rad and t2_rad <= 4.71238898038469:
        quadrant = 3
    elif 4.729842272904633 >= t2_rad and t2_rad <= 6.283185307179586:
        quadrant = 4
        
    return [[x,y,z],quadrant]

def RotationMatrix(xtheta,ytheta): 
    xmatrix = np.asarray([[1,0,0],[0,np.cos(xtheta),np.sin(xtheta)],[0,-np.sin(xtheta),np.cos(xtheta)]])
    ymatrix = np.asarray([[np.cos(ytheta),0,-np.sin(ytheta)],[0,1,0],[np.sin(ytheta),0,np.cos(ytheta)]])
    return xmatrix,ymatrix

def getSphericalCoords(targetP, radius):
    stack = []
    stack.append([[targetP[0],targetP[1],targetP[2]+radius],0])
    for i in [35,50,60]:
        for j in [15,45,120,180,300,360]:
            val = sphericalRelative(math.radians(i), math.radians(j), targetP, radius)
            stack.append(val)
    return stack

def getSphericalCoords1(targetP, radius,xdeg,ydeg):
    stack = []
    targetP1 =[targetP[0],targetP[1],targetP[2]+radius]
    stack.append([targetP1,0])
    stack.append([[targetP[0]+radius,targetP[1],targetP[2]],0])
    stack.append([[targetP[0],targetP[1]+radius,targetP[2]],0])
    vector = np.subtract(targetP1,targetP)
    unit_v = vector/np.linalg.norm(vector)
    print(unit_v)
    rx,ry = RotationMatrix(np.radians(xdeg),np.radians(ydeg))
    R = rx*ry
    print(rx,ry)
    print(R)
    
    rotvecx = np.dot(rx,unit_v)
    rotvecy = np.dot(ry,rotvecx)
    print("vector1",rotvecy)
    vecrot = np.dot(R,unit_v)

    point = targetP + radius*rotvecy
    print(point)
    stack.append([point,0])
    return stack

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return math.degrees(angle)






if __name__ == '__main__':

    rad = 10
    InpPoint = [433,221,332]
    tmpStack = np.asarray(getSphericalCoords1(InpPoint, rad,45,180)) # Choose some random 3d point for InpPoint
    allpnts = np.asarray(tmpStack[:,0])

    xp = []
    yp = []
    zp = []
    fig = go.Figure()
    for i in range(len(allpnts)-1):
        xp.append(allpnts[i][0])
        yp.append(allpnts[i][1])
        zp.append(allpnts[i][2])
        data = go.Scatter3d(x=[InpPoint[0],allpnts[i][0]],y=[InpPoint[1],allpnts[i][1]],z=[InpPoint[2],allpnts[i][2]])
        fig.add_trace(data)

    dist = np.linalg.norm((np.subtract(InpPoint,allpnts[3]))) 
    data1 = go.Scatter3d(x=[InpPoint[0],allpnts[3][0]],y=[InpPoint[1],allpnts[3][1]],z=[InpPoint[2],allpnts[3][2]],marker=dict(size=[50,50],color = ('rgb(22, 96, 167)')))
    fig.add_trace(data1)
    print(dist)
    print(allpnts[3])
    
    fig.show()