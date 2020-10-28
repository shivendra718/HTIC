import urx
import math
from math import *
import math3d as m3d
import numpy as np
import time
import plotly.graph_objects as go
import sys
import plotly.io as pio

def OrientMatixRotation(axis,rotationvec,theta):
    comp1 = rotationvec * np.cos(theta)
    comp2 = np.cross(rotationvec,axis) * np.sin(theta)
    comp3 = (rotationvec * (np.dot(rotationvec,axis)))* (1 - np.cos(theta))
    Rodrigues = comp1 + comp2 + comp3
    return Rodrigues

def FigVec(fig,vector1,vector2,vector3):
#     data = go.Scatter3d(x=[origin[0],vector1[0]],y=[origin[1],vector1[1]],z=[origin[2],vector1[2]],mode = 'markers', marker=dict(opacity = 0.5, color = ('rgb(204, 102, 0)')))
#     data1 = go.Scatter3d(x=[origin[0],vector2[0]],y=[origin[1],vector2[1]],z=[origin[2],vector2[2]],mode = 'markers', marker=dict(opacity = 0.5, color = ('rgb(204, 102, 0)')))
    data2 = go.Scatter3d(x=[vector3[0]],y=[vector3[1]],z=[vector3[2]],mode = 'markers', marker=dict(opacity = 0.5, color = ('rgb(204, 102, 0)')))
#     fig.add_trace(data)
#     fig.add_trace(data1)
    fig.add_trace(data2)
    return fig,[vector1,vector2,vector3]



R= np.matrix([[ 1,0,0],[0,1,0],[0,0,1 ]])
print(R)

origin = [0,0,0]
vectorx =np.array( [1,0,0])
vectory =np.array( [0,1,0])
vectorz =np.array( [0,0,1 ])

RotationMatrix = []

fig = go.Figure()
theta = np.linspace(0,2*pi,20)
phi = np.linspace(0,pi,20)
# print(theta)
for i in phi:
    print(i)
    for j in theta:
        Rod_y = OrientMatixRotation(vectorx,vectory,j)
        Rod_z = OrientMatixRotation(vectorx,vectorz,j)
        fig,r = FigVec(fig,vectorx,Rod_y,Rod_z)
        R1 = np.asarray([vectorx,Rod_y,Rod_z])
        RotationMatrix.append(R1)
#         R1 = EulerAngle(Rod_y[0],Rod_y[1],Rod_y[2])
#         RMatrix = rotsplit(R1)
#         RotationMatrix.append(RMatrix)
#         R2 = EulerAngle(Rod_z[0],Rod_z[1],Rod_z[2])
#         RMatrix1 = rotsplit(R2)
#         RotationMatrix.append(RMatrix1)
        
        
        
#         for k in range(len(r)): 
#                 R1 = EulerAngle(r[k][0],r[k][1],r[k][2])
#                 vectorx1 = np.asarray([R1[0,0],R1[1,0],R1[2,0]])
#                 vectory1 = np.asarray([R1[0,1],R1[1,1],R1[2,1]])
#                 vectorz1 = np.asarray([R1[0,2],R1[1,2],R1[2,2]])
#                 RMatrix = np.asarray([vectorx1,vectory1,vectorz1])
#                 RotationMatrix.append(RMatrix)
                
    Rod_xy = OrientMatixRotation(Rod_y,vectorx,i)
    Rod_xz = OrientMatixRotation(Rod_y,Rod_z,i)
    
    vectorx = Rod_xy
    vectory = vectory
    vectorz = Rod_xz
#     R= np.asarray([Rod_xy,vectory,Rod_xz])
# #   fig,r = FigVec(fig,Rod_xy,vectory,Rod_xz)
# #         print(R)
#     RotationMatrix.append(np.transpose(R))
#     vectorx = np.asarray([R[0,0],R[1,0],R[2,0]])
#     vectory = np.asarray([R[0,1],R[1,1],R[2,1]])
#     vectorz = np.asarray([R[0,2],R[1,2],R[2,2]])
#     fig,r = FigVec(fig,vectorx,vectory,vectorz)
    
#         fig, r = FigVec(fig,vectorx,vectory,vectorz)
        
#         fig,r1 = FigVec(fig,vectorx,Rod_y,Rod_z)
        
# #         fig = FigVec(fig,vectorx,Rod_y,vectorz)
#         Rod_xy = OrientMatixRotation(Rod_y,vectorx,phi[j])
# #         print(Rod_xy)
# #         Rod_y1 = OrientMatixRotation(Rod_xy,Rod_y,theta[i])
#         Rod_y1 = OrientMatixRotation(Rod_xy,Rod_y,theta[i])
# #         print(Rod_xy)
# #         print(Rod_y1)
# #         print(vectorz)
# #         fig,r = FigVec(fig,Rod_xy,Rod_y1,vectorz)
#         fig,r = FigVec(fig,Rod_y,Rod_xy,Rod_y1)
# #         print(r)
# print(Rod_xy)          
# R1 = EulerAngle(-0.01852074617501981,-0.11105406264394302,-0.9936417750535276)
# print(r)
# print(R1)
# print(RotationMatrix)
# print(fig.data)
# print(len(phi),phi)
# print(len(RotationMatrix))
# # print(RotationMatrix)
fig.show()