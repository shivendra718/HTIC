import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm

start_end = []
radius = []
step_ = []
vector_ = []
unitvector_ = []
distance_ = []

fig = go.Figure()

def Cylinder(pt1,pt2,r):
    v = subtract(pt2,pt1)
    vector_.append(v)
    mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    distance_.append(mag)
    unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
    unit_v=asarray(unit_v)
    unitvector_.append(unit_v)
    dist = sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2 + (pt2[2]-pt1[2])**2)
    v1 = pt1
    v2 = pt2

    nt = 100
    nv = 50
    mag_dist = norm(v)
    not_v=[1,0,0]
    
    if (unit_v == not_v).all():
        not_v = np.array([0, 1, 0])
    n1 = np.cross(unit_v, not_v)
    n1 /= norm(n1)
    n2 = np.cross(unit_v, n1)

    theta = np.linspace(0, 2*np.pi, nt)
    phi = linspace(0,np.pi,100)
    v = np.linspace(0, mag_dist, nv )
    rsample=np.linspace(0,r,2)
    rsample,theta = np.meshgrid(rsample,theta)
    theta1, v = np.meshgrid(theta, v)

    x,y,z = [v1[i] + unit_v[i] * v + r * np.sin(theta1) * n1[i] + r * np.cos(theta1) * n2[i] for i in [0, 1, 2]] 
    
    
    cyl1 = go.Surface(x=x, y=y, z=z,
                 showscale=False,
                 opacity=0.5)
    
    fig.add_trace(cyl1)
    start_end.append([v1,v2])
    radius.append(r)
    return fig


fig = Cylinder([5,7,2],[8,3,6],1.6)
fig = Cylinder([12,12,12],[7,7,7],1.6)
# fig = Cylinder([8,3,6],[5,7,2],1.6)
fig.show()

cross_v = np.cross(vector_[0],vector_[1])
unitvector = cross_v/norm(cross_v)
vector12 = np.subtract(start_end[0][0],start_end[1][0])
distance12 = norm(vector12)
vector21 = np.subtract(start_end[0][1],start_end[1][1])
distance21 = norm(vector21)
# print(distance12,distance21)
if not(np.isnan(unitvector[0]) and np.isnan(unitvector[1]) and np.isnan(unitvector[2])):
    d = norm((unitvector*vector12))
    d_cylinder1 = distance12 - (radius[0] + radius[1])
    d_cylinder2 = distance21 - (radius[0] + radius[1])
    print(d_cylinder1)

    # if (d_cylinder == 0):
    #     print("Interference possible")
 
    if ((d_cylinder1) < (radius[0]+radius[1])):
        print("non-parallel and interference possible")

    # elif (d_cylinder2) < (radius[0]+radius[1]):
    #     print("non-parallel and interference possible") 
    else:
        print("no interference")
else:
    print("parallel")