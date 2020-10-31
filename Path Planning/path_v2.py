# In this code a and b are the two points, h is the maxium height of the curve and div is the number of divisions of the curve including the 1st and last point.

import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm



fig = go.Figure()

a = [5,1]
b = [25,1]
h = 10
div = 50
pt1 = [8,1]
pt2 = [6,2]


distance = []
c = [(a[0]+b[0])/2,(a[1]+b[1])/2]
data = go.Scatter(x=[a[0],b[0]], y=[a[1],b[1]])



# vector1 = np.subtract(pt2,pt1)
# mag1 = norm(vector1)
# print(vector1)
# unitv1 = [vector1[0]/mag1,vector1[1]/mag1]
# unitv1 = np.asarray(unitv1)
# unitmag = norm(unitv1)
# l1 = pt1 + (mag1*unitv1)
# # l1 = [l1[0],l1[1],1]
# l1min = pt1[0]
# l1max = pt1[1]
# L1 = linspace(l1min,l1max,div)
# datan = go.Scatter(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]])
# fig.add_trace(datan)
# vector_r = np.subtract(c,b)
# r = norm(vector_r)




tmin = 0
tmax = np.pi
cmin = a[0]
cmax = b[0]
bmin = a[1]
bmax = b[1]
C = list(linspace(cmin, cmax,div))
t = (linspace(tmin, tmax,div))
B = list(linspace(bmin, bmax,div))
y = (h)*(sin((t)) ) + B
x = cos(t) + C
T = pi
nmin = ceil(tmin / T)
nmax = floor(tmax / T)
n = linspace(nmin,nmax,div)
y1 = (h)*(sin((n*T)) ) + B

# for i in range(0,len(C)-1):
#     if C[i] < pt1[0] and C[i+1] > pt1[0]:
#         curveNormal = np.array([0, 1])
#         Y = (y[i]+y[i+1])/2 
#         curvePoint = np.array([pt1[0],Y]) #Any point on the plane


#         w = np.subtract(pt1,curvePoint)

#         N = -np.dot(curveNormal,w)
#         D = np.dot(curveNormal,l1)
#         sI = N/D
#         I=pt1+(sI*l1)
#         print(I)
#         I[0] = pt1[0]
#         datan = go.Scatter(x=[pt1[0],I[0]], y=[pt1[1],I[1]])
#         fig.add_trace(datan)
#         dist = np.sqrt((pt1[0]-I[0])**2 + (pt1[1]-I[1])**2)
#         distance.append(dist)
#         print(dist)

for i in range(0,len(C)-1):    
    if C[i] < pt1[0] and C[i+1] > pt1[0]:
        magni = np.sqrt((C[i]-pt1[0])**2 + (B[i]-pt1[1])**2)
        magni1 = np.sqrt((C[i]-C[i+1])**2 + (y1[i]-y1[i+1])**2)
        p1 = [C[i],y1[i]]
        p2 = [C[i+1],y1[i+1]]
        v = np.subtract(p2,p1)
        scale = (magni/magni1)
        d = magni1*scale
        l2 = p1 + (d*(v))
        datan = go.Scatter(x=[pt1[0],l2[0]], y=[pt1[1],l2[1]])
        fig.add_trace(datan) 
        

# print(C)
# p1 = [9.210526,7.142127,1]
# p2 = [10.26316,8.357239,1]
# pt1 = [9.5,1,1]
# pt2 = [9.5,2,1]
# crosspt = np.cross(pt1,pt2)
# crossp = np.cross(p1,p2)
# print(crossp)
# crossv = np.cross(crossp,crosspt)
# print(crossv)
# datan = go.Scatter(x=[pt1[0],crossv[0]], y=[pt1[1],crossv[1]])
# fig.add_trace(datan)

surf = go.Scatter(x=C,y=y)
# surf1 = go.Scatter(x=C,y=y1,mode = "markers")
# for i in range(0,len(C)):
#     data1 = go.Scatter(x=[C[i],C[i]],y=[y1[i],B[i]])
#     fig.add_trace(data1)
    # if l1[0] == C[i] and l1[i] == y1[i]:
    #     dist = np.sqrt((C[i]-C[i])**2 + (y1[i]-B[i])**2)
    #     distance.append(dist)

fig.add_trace(surf)
fig.add_trace(data)
print(distance)


fig.show()