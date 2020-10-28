import urx
import math
from math import *
import math3d as m3d
import numpy as np
import time
import plotly.graph_objects as go
import pandas as pd

# #Connection Establishment
# robo = urx.Robot("127.0.0.1")

def JointLocations(thetas):
    
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997 
    d6 = 0.0996 
    
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
    
    theta1 = [0,0,d1]
    
    theta2 = [(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]
               
    theta3 = [np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]
               
    theta4 = [(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]
               
    theta5 = [ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))]
               
    theta6 = [((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))]
    
    positions = [theta1,theta2,theta3,theta4,theta5,theta6]
    return positions

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

def EulerAngle(xtheta,ytheta,ztheta):
    xmatrix = np.matrix([[1,0,0],[0,cos(xtheta),-sin(xtheta)],[0,sin(xtheta),cos(xtheta)]])
    ymatrix = np.matrix([[cos(ytheta),0,sin(ytheta)],[0,1,0],[-sin(ytheta),0,cos(ytheta)]])
    zmatrix = np.matrix([[cos(ztheta),-sin(ztheta),0],[sin(ztheta),cos(ztheta),0],[0,0,1]])
    EulerAngleRot = zmatrix*ymatrix*xmatrix
    return EulerAngleRot

def OrientMatixRotation(axis,rotationvec,theta):
    comp1 = rotationvec * np.cos(theta)
    comp2 = np.cross(rotationvec,axis) * np.sin(theta)
    comp3 = (rotationvec * (np.dot(rotationvec,axis)))* (1 - np.cos(theta))
    Rodrigues = comp1 + comp2 + comp3
    return Rodrigues

def RobotBaseCylinder(r, h, a =0, nt=100, nv =50):
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = 0+r*np.cos(theta)
    y= 0+r*np.sin(theta)
    z = 0+v
    return x, y, z

def RobotBaseStructure():
    thetas = [0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
    temp_joint_locations = thetas
    sl = JointLocations(temp_joint_locations)
    x_p,y_p,z_p = SepPoints(sl)

    x_p.insert(1,0)
    x_p.insert(2,-2.602374e-17)

    y_p.insert(1,-0.1333)
    y_p.insert(2,-0.1333)

    z_p.insert(1,0.1625)
    z_p.insert(2,0.5875)    


    x_p=list(np.asarray(x_p) * 1000)
    y_p=list(np.asarray(y_p) * 1000)
    z_p=list(np.asarray(z_p) * 1000)

    data=go.Scatter3d(x=x_p,
                        y=y_p,
                        z=z_p,marker=dict(
                size=[40,40,36,36,28,28,28,28],
                opacity = 0,
                color = ('rgb(22, 96, 167)'),              
                colorscale='Viridis',  

            ),
                line = dict(
                colorscale="Viridis",
                width = 50,
                ),
            )
    fig = go.Figure(data = data)
    fig.update_layout(scene=dict(zaxis=dict(range=[-1200,1200],autorange=False),       
                        yaxis=dict(range=[-1200,1200],autorange=False),
                    xaxis=dict(range=[-1200,1200],autorange=False)))               
    fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

    fig.add_trace(data)
   
    return fig

def DHTable2HomTrans(DHTable):
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i in range(np.shape(DHTable)[0]):
        T1 = np.dot(T,DH2HomTrans(DHTable[i]))
        T = T1
    return T

def DH2HomTrans(DHparams):
    [al,a,d,th] = DHparams
    T = [[np.cos(th),-np.sin(th)*np.cos(al),np.sin(th)*np.sin(al),a*np.cos(th)],[np.sin(th),np.cos(th)*np.cos(al),-np.cos(th)*np.sin(al),a*np.sin(th)],[0,np.sin(al),np.cos(al),d],[0,0,0,1]]
    return T

def sol2Rik(x,y,l1,l2):
    a = -2.0*l1*x
    b = -2.0*l1*y
    c = l1**2.0 - l2**2.0 + x**2.0 + y**2.0
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta2 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta12 = np.arctan2((y - l1*np.sin(i))/l2,(x - l1*np.cos(i))/l2)
        theta2[j] = theta12 - i
        j = j+1
    t1t2 = np.column_stack((theta1, theta2))
    return t1t2

def solUR5ik(p,R):
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997 #+ 0.025
    d6 = 0.0996 #+ 0.08165
    [x,y,z] = p
    [[nx,ox,ax],[ny,oy,ay],[nz,oz,az]] = R
    a = ay*d6 - y
    b = -ax*d6 + x
    c = -d4
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta5 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta5[j] = [np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6),-np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6)]
        j = j+1
    
    
    t1t5 = [[theta1[0],theta5[0][0]],[theta1[0],theta5[0][1]],[theta1[1],theta5[1][0]],[theta1[1],theta5[1][1]]]
    theta6 = [None]*np.shape(t1t5)[0]
    j = 0
    for i in range(np.shape(t1t5)[0]):
        theta6[j] = [np.arctan2((oy*np.cos(t1t5[i][0])-ox*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]),(-ny*np.cos(t1t5[i][0])+nx*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]))]
        j = j+1     
    t1t5t6 = np.hstack((t1t5,theta6))
    k = 0
    t2t3t4 = [None]*np.shape(t1t5)[0]*2  
    TUR5 = np.vstack((np.hstack((R,np.array(p).reshape(3,1))),[0,0,0,1]))
    for i in range(np.shape(t1t5t6)[0]):
        T01 = DHTable2HomTrans([[np.pi/2,0,d1,t1t5t6[i][0]]])
        T56 = DHTable2HomTrans([[-np.pi/2,0,d5,t1t5t6[i][1]],[0,0,d6,t1t5t6[i][2]]])
        T3R = np.dot(np.dot(np.linalg.inv(T01),TUR5),np.linalg.inv(T56))
        
        theta234 = np.arctan2(T3R[1][0],T3R[0][0])
        theta23 = sol2Rik(T3R[0][3],T3R[1][3],a2,a3)
        for j in range(np.shape(theta23)[0]):
            theta4 = theta234 - theta23[j][0] - theta23[j][1]
            t2t3t4[k]=[theta23[j][0],theta23[j][1],theta4]
            k = k+1
    t1 = np.array([val for val in t1t5 for _ in (0, 1)])[:,0]
    t5 = np.array([val for val in t1t5 for _ in (0, 1)])[:,1]
    t6 = np.array([val for val in theta6 for _ in (0, 1)])
    ikUR5 = np.hstack((t1.reshape(8,1),t2t3t4,t5.reshape(8,1),t6.reshape(8,1)))
    return ikUR5

def traceremoval(fig,data):
    l = list(fig.data)
    l.remove(data)
    fig.data = tuple(l)
    return fig.data

def FigVec(fig,vector1,vector2,vector3):
    data = go.Scatter3d(x=[origin[0],vector1[0]],y=[origin[1],vector1[1]],z=[origin[2],vector1[2]])
    data1 = go.Scatter3d(x=[origin[0],vector2[0]],y=[origin[1],vector2[1]],z=[origin[2],vector2[2]])
    data2 = go.Scatter3d(x=[origin[0],vector3[0]],y=[origin[1],vector3[1]],z=[origin[2],vector3[2]])
    fig.add_trace(data)
    fig.add_trace(data1)
    fig.add_trace(data2)
    return fig,[vector1,vector2,vector3]

## MAIN   
if __name__ == '__main__':

    # current_data = [-0.4918001388878259, -0.1333000325950433, 0.48779999999306556, -1.2091996762909452, -1.209199653068142, 1.2092000716991698]
    current_data =  [-0.09178279996218568, -0.4356200606336901, 0.20203002541831916, -0.0012213596819496176, 3.116276528472194, 0.03889191565163747]
    step = 10
    fig = go.Figure()

    xtheta = current_data[3]
    ytheta = current_data[4]
    ztheta = current_data[5]
    #### orientation
    # R= np.matrix([[ -0.1388207, -0.7002603, -0.7002601],[0.7002603,  0.4305894, -0.5694104],[0.7002601, -0.5694104,  0.4305898 ]])
    R = R= np.matrix([[ -0.0000003, -0.0000000, -1.0000000],[1.0000000, -0.0000003, -0.0000003],[-0.0000003, -1.0000000,  0.0000000 ]])
    RotationMatrix = []
    origin = [0,0,0]
    vectorx = np.asarray([R[0,0],R[1,0],R[2,0]])
    vectory = np.asarray([R[0,1],R[1,1],R[2,1]])
    vectorz = np.asarray([R[0,2],R[1,2],R[2,2]])

    theta = np.linspace(0,2*pi,20)
    phi = np.linspace(0,pi,10)
    for i in range(0,len(theta)):
        for j in range(0,len(phi)):
            Rod_y = OrientMatixRotation(vectorx,vectory,theta[i])
    #         fig = FigVec(fig,vectorx,Rod_y,vectorz)
            Rod_xy = OrientMatixRotation(Rod_y,vectorx,phi[j])
            Rod_y1 = OrientMatixRotation(Rod_xy,Rod_y,theta[i])
            fig,r = FigVec(fig,Rod_xy,Rod_y1,vectorz)
            for k in range(len(r)): 
                R1 = EulerAngle(r[k][0],r[k][1],r[k][2])
                vectorx1 = np.asarray([R1[0,0],R1[1,0],R1[2,0]])
                vectory1 = np.asarray([R1[0,1],R1[1,1],R1[2,1]])
                vectorz1 = np.asarray([R1[0,2],R1[1,2],R1[2,2]])
                RMatrix = np.asarray([vectorx1,vectory1,vectorz1])
                RotationMatrix.append(RMatrix)

    # print(RotationMatrix)       

##### position
    p1 = np.asarray([-0.4915471811910556, -0.13330003259498632, 0.7794309980434397])*1000
    p2 = np.asarray([-0.69180013875264156, 0.33330003259504258, -0.01071533333064798])*1000
    p3 = np.asarray([-0.3592309116311874, -0.38214092920307395, 0.06871533334326907])*1000
    p4 = np.asarray([-0.3592309116311874,0.33330003259504258,  0.7794309980434397])*1000

    xmin = -0.69180013875264156*1000
    # xmax = 0.69180013875264156*1000
    xmax = 0
    ymin = -0.75014092920307395*1000
    ymax = 0.75014092920307395*1000
    zmin = -0.01071533333064798*1000
    zmax = 1.07794309980434397*1000

    x1 = np.linspace(xmin,xmax,step)
    y1 = np.linspace(ymin,ymax,step)
    z1 = np.linspace(zmin,zmax,step)
    
    xpts = []    ### complete cuboid points
    ypts = []
    zpts = []
    xptsreach = []  ### reachabble workspace points
    yptsreach = []
    zptsreach = []
    xptsd = []   ### dspace points
    yptsd = []
    zptsd = []


    for i in range(len(x1)):
        for j in range(len(y1)):
            flag_dspace = 0
            flag_out = 0
            flag_reachable = 0
            for k in range(len(z1)):
                xcuboid = np.asarray(x1[i])/1000
                ycuboid = np.asarray(y1[j])/1000
                zcuboid = np.asarray(z1[k])/1000
                p = np.asarray([xcuboid,ycuboid,zcuboid])
                xpts += [x1[i]]
                ypts += [y1[j]]
                zpts += [z1[k]]
                for l in range(len(RotationMatrix)):
                    ik = solUR5ik(p,RotationMatrix[l])
                    if not (np.isnan(ik[0]).any() or np.isnan(ik[1]).any() or np.isnan(ik[2]).any() or np.isnan(ik[3]).any() or np.isnan(ik[4]).any() or np.isnan(ik[5]).any() or np.isnan(ik[6]).any() or np.isnan(ik[7]).any()):
                        flag_dspace += 1
                        print(ik)
                    else :
                        if np.isnan(ik[0]).all() and np.isnan(ik[1]).all() and np.isnan(ik[2]).all() and np.isnan(ik[3]).all() and np.isnan(ik[4]).all() and np.isnan(ik[5]).all() and np.isnan(ik[6]).all() and np.isnan(ik[7]).all():
                            flag_out += 1
                        else:
                            flag_reachable += 1

                if flag_dspace == len(RotationMatrix):
                    xptsd += [x1[i]]
                    yptsd += [y1[j]]
                    zptsd += [z1[k]]
                if flag_out == len(RotationMatrix):
                    pass
                if flag_reachable == len(RotationMatrix):
                    xptsreach += [x1[i]]
                    yptsreach += [y1[j]]
                    zptsreach += [z1[k]]

    columns = ['xptsd','yptsd','zptsd']
    columns1 = ['xpts','ypts','zpts']
    columns2 = ['xptsreach','yptsreach','zptsreach']
    # data = [xptsd,yptsd,zptsd]
    # print(data)
    print(xptsd)
    df=pd.DataFrame(data=[xptsd,yptsd,zptsd],index=columns).T
    df.to_csv('data.csv')
    df1 = pd.DataFrame(data=[xpts,ypts,zpts],index=columns1).T
    df1.to_csv('data1.csv')
    df2 = pd.DataFrame(data=[xptsreach,yptsreach,zptsreach],index=columns2).T
    df2.to_csv('data2.csv')
    
    # data = go.Scatter3d(x=xpts,y=ypts,z=zpts,mode = 'markers', marker=dict(opacity = 0.5, color = ('rgb(204, 102, 0)')))
    # fig.add_trace(data)
    # fig.show()
    # data1 = go.Scatter3d(x=xptsd,y=yptsd,z=zptsd)
    # data2 = go.Scatter3d(x=xptsreach,y=yptsreach,z=zptsreach)
    # fig.add_trace(data1)
    # fig.show()
    # fig.data = traceremoval(fig,data)
    # fig.show()

   