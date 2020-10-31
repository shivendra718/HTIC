import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm

pt1 = [0,0,0]
pt2 = [0,0,5]
r = 1.6
fig = go.Figure()

data = go.Scatter3d(x =[pt1[0],pt2[0]],y=[pt1[1],pt2[1]],z=[pt1[2],pt2[2]])
fig.add_trace(data)

v = subtract(pt2,pt1)
mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
unit_v=asarray(unit_v)
# dist = sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2 + (pt2[2]-pt1[2])**2)
# v1 = pt1
# v2 = pt2
# nt = 100
# nv = 50
mag_dist = norm(v)
not_v=[1,0,0]
    
if (unit_v == not_v).all():
    not_v = np.array([0, 1, 0])
n1 = np.cross(unit_v, not_v)
n1 /= norm(n1)
n2 = np.cross(unit_v, n1)

# theta = np.linspace(0, 2*np.pi, nt)
# phi = linspace(0,np.pi,100)
# v = np.linspace(0, mag_dist, nv )
div = 100
# rsample=np.linspace(0,r,2)
theta = np.linspace(0, 2*np.pi, div)
# rsample,theta = np.meshgrid(rsample,theta)
# theta1, v = np.meshgrid(theta, v)

# x, y, z = [pt1[i] + r * np.sin(theta) * n1[i] + r * np.cos(theta) * n2[i] for i in [0, 1, 2]]
# print(x,y,z)

# div = 100
# theta = np.linspace(0, 2*np.pi, div)
x = pt1[0] + r * np.sin(theta) * n1[0] + r * np.cos(theta) * n2[0]
y = pt1[1] + r * np.sin(theta) * n1[1] + r * np.cos(theta) * n2[1]
z = pt1[2] + r * np.sin(theta) * n1[2] + r * np.cos(theta) * n2[2]
# x, y, z = [pt1[i] + r * np.sin(theta) * n1[i] + r * np.cos(theta) * n2[i] for i in [0, 1, 2]]
# z = pt1[2]+np.ones(theta.shape)
# z = pt1[2]-np.cos(theta)

# x1 = pt2[0]+r*np.cos(theta)
# y1 = pt2[1]+r*np.sin(theta)
# z1 = pt2[2]-np.cos(theta)
x1, y1, z1 = [pt2[i] + r * np.sin(theta) * n1[i] + r * np.cos(theta) * n2[i] for i in [0, 1, 2]]
area = pi*r**2
pt_1 = [pt1[0]+r,pt1[1]+r,pt1[2]+r]
pt_2 = [pt2[0]+r,pt2[1]+r,pt2[2]+r]
# data2 = go.Scatter3d(x=[pt1[0],pt_1[0]],y=[pt1[1],pt_1[1]],z=[pt1[2],pt_1[2]])
# fig.add_trace(data2)
for i in range(div):
    x_1 = x[i]
    y_1 = y[i]
    z_1 = z[i]
    x_2 = x1[i]
    y_2 = y1[i]
    z_2 = z1[i]
    data2=go.Scatter3d(x=[x_1,x_2],y=[y_1,y_2],z=[z_1,z_2])
    fig.add_trace(data2)    





bcircle1 = go.Scatter3d(x=x,y=y,z=z)
fig.add_trace(bcircle1)
# bcircle2 = go.Scatter3d(x=x1,y=y1,z=z1)
# fig.add_trace(bcircle2)
fig.show()



# def Cylinder(pt1,pt2,r):
#     step = 7
#     data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]])
    
#     v = subtract(pt2,pt1)
#     mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
#     unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
#     unit_v=asarray(unit_v)
#     dist = sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2 + (pt2[2]-pt1[2])**2)
#     v1 = pt1
#     v2 = pt2
#     pdist = dist/step
#     newpts_array = []
#     for i in range(0, step):
#         dist_v = pdist*unit_v
#         newpts = pt1 + dist_v
#         newpts_array.append(newpts)
#         pt1 = newpts
#         newpts=list(newpts)
#         theta = linspace(0,2*np.pi,100)
#         phi = linspace(0,np.pi,100)

#         x = newpts[0]+r*outer(cos(theta),sin(phi))
#         y = newpts[1]+r*outer(sin(theta),sin(phi))    
#         z = newpts[2]+r*outer(ones(100),cos(phi))

#         data=go.Surface(
#             x=x,
#             y=y,
#             z=z,
#             opacity=0.3
#         )
#         fig.add_trace(data)

#     nt = 100
#     nv = 50
#     mag_dist = norm(v)
#     not_v=[1,0,0]
    
#     if (unit_v == not_v).all():
#         not_v = np.array([0, 1, 0])
#     n1 = np.cross(unit_v, not_v)
#     n1 /= norm(n1)
#     n2 = np.cross(unit_v, n1)

#     theta = np.linspace(0, 2*np.pi, nt)
#     phi = linspace(0,np.pi,100)
#     v = np.linspace(0, mag_dist, nv )
#     rsample=np.linspace(0,r,2)
#     rsample,theta = np.meshgrid(rsample,theta)
#     theta1, v = np.meshgrid(theta, v)

#     x,y,z = [v1[i] + unit_v[i] * v + r * np.sin(theta1) * n1[i] + r * np.cos(theta1) * n2[i] for i in [0, 1, 2]] 
    
    
#     cyl1 = go.Surface(x=x, y=y, z=z,
#                  showscale=False,
#                  opacity=0.5)
#     fig.add_trace(cyl1)

#     start_end.append([v1,v2])
#     radius.append(r)
#     step_.append(step)
#     return fig