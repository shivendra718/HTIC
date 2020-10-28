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

def RotationMatrix(xtheta,ytheta,ztheta): 
    xmatrix = np.asarray([[1,0,0],[0,np.cos(xtheta),np.sin(xtheta)],[0,-np.sin(xtheta),np.cos(xtheta)]])
    ymatrix = np.asarray([[np.cos(ytheta),0,-np.sin(ytheta)],[0,1,0],[np.sin(ytheta),0,np.cos(ytheta)]])
    zmatrix = np.asarray([[np.cos(ztheta),-np.sin(ztheta),0],[np.sin(ztheta),np.cos(ztheta),0],[0,0,1]])
    return xmatrix,ymatrix,zmatrix

def getSphericalCoords1(targetP, radius,xdeg,ydeg):
    stack = []
    targetP1 =[targetP[0],targetP[1],targetP[2]+radius]
    stack.append([targetP1,0])
    stack.append([[targetP[0]+radius,targetP[1],targetP[2]],0])
    stack.append([[targetP[0],targetP[1]+radius,targetP[2]],0])
    vector = np.subtract(targetP1,targetP)
    unit_v = vector/np.linalg.norm(vector)
    # print(unit_v)
    rx,ry,rz = RotationMatrix(np.radians(xdeg),np.radians(ydeg),np.radians(ydeg))
    R = rx*rz
    # print(rx,rz)
    # print(R)
    
    rotvecx = np.dot(rx,unit_v)
    rotvecy = np.dot(rz,rotvecx)
    # print("vector1",rotvecy)
    vecrot = np.dot(R,unit_v)

    point = targetP + radius*rotvecy
    # print(point)
    stack.append([point,0])
    return stack

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return math.degrees(angle)

# def Entry(fig,target,radius,divrot1,divrot2):
def Entry(fig,target,radius):
    allp1 = []
    allp2 = []
    # divrot1 = int(360/divrot1)
    # divrot2 = int(90/divrot2)
    rot1 = [30,45,65]
    rot2 = [60,120,180,240,300,360]
    for i in rot1:
        for j in rot2:
            tmpStack = np.asarray(getSphericalCoords1(target, radius,i,j)) # Choose some random 3d point for InpPoint
            allpnts = np.asarray(tmpStack[:,0])
            print(i,j)
            allp1.append(list(allpnts))
            # allp2.append(allpnts[3])

    # print(xp,yp,zp)
    for j in range(len(allp1)):
        for k in range(len(allp1[0])):
            data = go.Scatter3d(x=[InpPoint[0],allp1[j][k][0]],y=[InpPoint[1],allp1[j][k][1]],z=[InpPoint[2],allp1[j][k][2]])
            fig.add_trace(data)
    return np.asarray(allp1)[:,3], fig

if __name__ == '__main__':

    rad = 10
    InpPoint = [ -611.32716628,  -139.97226391,  -204.29798313]
    fig = go.Figure()

    allp1,fig = Entry(fig,InpPoint,rad)
    print(list(allp1))