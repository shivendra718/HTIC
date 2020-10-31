# In this code a and b are the two points, h is the maxium height of the curve and div is the number of divisions of the curve including the 1st and last point.

import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm

fig = go.Figure()

def path(a,b,h,pt1):
    global distance
    distance = []
    c = [(a[0]+b[0])/2,(a[1]+b[1])/2]
    data = go.Scatter(x=[a[0],b[0]], y=[a[1],b[1]])
    vector = np.subtract(b,a)
    dist = norm(vector) 
    div = dist*5
    tmin = 0
    tmax = np.pi
    cmin = a[0]
    cmax = b[0]
    bmin = a[1]
    bmax = b[1]
    C = linspace(cmin, cmax, div)
    t = linspace(tmin, tmax, div)
    B = linspace(bmin, bmax, div)
    y = (h)*(sin((t)) ) + B


    for i in range(0,len(C)-1):
        for j in range(0,len(pt1)):
            if C[i] <= pt1[j][0] and C[i+1] >= pt1[j][0]:
                magni = np.sqrt((C[i]-pt1[j][0])**2 + (B[i]-pt1[j][1])**2)
                magni1 = np.sqrt((C[i]-C[i+1])**2 + (y[i]-y[i+1])**2)
                p1 = [C[i],y[i]]
                p2 = [C[i+1],y[i+1]]
                v = np.subtract(p2,p1)
                scale = (magni/magni1)
                d = scale + magni 
                l2 = p1 + (d*(v))
                if pt1[j] == b:
                    l2 = b
                datan = go.Scatter(x=[pt1[j][0],l2[0]], y=[pt1[j][1],l2[1]])
                fig.add_trace(datan) 
                dist = np.sqrt((pt1[j][0]-l2[0])**2 + (pt1[j][1]-l2[1])**2)
                distance.append(dist)


    surf = go.Scatter(x=C,y=y)

    fig.add_trace(surf)
    fig.add_trace(data)
    print(distance)
    return fig



# fig = path([0,0],[15,0],50,[[8,0],[1,0],[5,0],[8.5,0],[14,0],[14.2,0],[15,0],[16,0],[17.3,0],[28,0],[130,0],[142,0],[30,0]])
fig = path([-300,-100,200],[-491.8,-133.3,487.8],50,[[-410,-120],[1,0],[2,0],[3,0],[4,0],[5,0],[14.2,0],[14.8,0],[100,0],[120,0],[130,0],[200,0],[220,0],[230,0],[300,0]])
fig.show()





