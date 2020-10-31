# import numpy as np
# import plotly.graph_objects as go
# from numpy import *
# from scipy.linalg import norm

# # fig = go.Figure()

# def cylinder_line(fig,pt1,pt2,r,var = True):
#     if pt1[2] == pt2[2] and pt1[1] == pt2[1]:
#         if pt1[0]>pt2[0]:
#             pt1[0],pt2[0] = pt2[0],pt1[0]

#     data = go.Scatter3d(x =[pt1[0],pt2[0]],y=[pt1[1],pt2[1]],z=[pt1[2],pt2[2]])
#     fig.add_trace(data)


#     div = 7

#     v = subtract(pt2,pt1)
#     mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
#     unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
#     unit_v = asarray(unit_v)

#     not_v = [1,0,0]
        
#     if (unit_v == not_v).all():
#         not_v = np.array([0, 1, 0])

    
#     n1 = np.cross(unit_v, not_v)
#     n1 /= norm(n1)
#     n2 = np.cross(unit_v, n1)

#     theta = np.linspace(0, 2*np.pi, div)

#     x = pt1[0] + r * np.sin(theta) * n1[0] + r * np.cos(theta) * n2[0]
#     y = pt1[1] + r * np.sin(theta) * n1[1] + r * np.cos(theta) * n2[1]
#     z = pt1[2] + r * np.sin(theta) * n1[2] + r * np.cos(theta) * n2[2]

#     x1 = pt2[0] + r * np.sin(theta) * n1[0] + r * np.cos(theta) * n2[0]
#     y1 = pt2[1] + r * np.sin(theta) * n1[1] + r * np.cos(theta) * n2[1]
#     z1 = pt2[2] + r * np.sin(theta) * n1[2] + r * np.cos(theta) * n2[2]

#     X1 = Y1 = Z1 = X2 = Y2 = Z2 = []

#     for i in range(div):
        
#         x_1 = x[i]
#         X1.append(x_1)
#         y_1 = y[i]
#         Y1.append(y_1)
#         z_1 = z[i]
#         Z1.append(z_1)
#         x_2 = x1[i]
#         X2.append(x_2)
#         y_2 = y1[i]
#         Y2.append(y_2) 
#         z_2 = z1[i]
#         Z2.append(z_2)
#         data2=go.Scatter3d(x=[x_1,x_2],y=[y_1,y_2],z=[z_1,z_2])
#         fig.add_trace(data2)
        


#     bcircle1 = go.Scatter3d(x=x,y=y,z=z)
#     fig.add_trace(bcircle1)
#     bcircle2 = go.Scatter3d(x=x1,y=y1,z=z1)
#     fig.add_trace(bcircle2)
#     if var == True:
#         return fig
#     else:
#         return X1,Y1,Z1,X2,Y2,Z2


# X1 = cylinder_line(fig,[0,0,162.5],[0,-133.3,162.5],1.6)
# print(X1)

# fig.show()

import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm


def cylinder_line(pt1,pt2,r):
    if pt1[2] == pt2[2] and pt1[1] == pt2[1]:
        if pt1[0]>pt2[0]:
            pt1[0],pt2[0] = pt2[0],pt1[0]

    div = 7

    v = subtract(pt2,pt1)
    mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
    unit_v = asarray(unit_v)

    not_v = [1,0,0]
        
    if (unit_v == not_v).all():
        not_v = np.array([0, 1, 0])

    
    n1 = np.cross(unit_v, not_v)
    n1 /= norm(n1)
    n2 = np.cross(unit_v, n1)

    theta = np.linspace(0, 2*np.pi, div)

    x = pt1[0] + r * np.sin(theta) * n1[0] + r * np.cos(theta) * n2[0]
    y = pt1[1] + r * np.sin(theta) * n1[1] + r * np.cos(theta) * n2[1]
    z = pt1[2] + r * np.sin(theta) * n1[2] + r * np.cos(theta) * n2[2]

    

    x1 = pt2[0] + r * np.sin(theta) * n1[0] + r * np.cos(theta) * n2[0]
    y1 = pt2[1] + r * np.sin(theta) * n1[1] + r * np.cos(theta) * n2[1]
    z1 = pt2[2] + r * np.sin(theta) * n1[2] + r * np.cos(theta) * n2[2]

    X1 = []
    Y1 = []
    Z1 = []
    X2 = []
    Y2 = []
    Z2 = []

    for i in range(div):
        
        x_1 = x[i]
        X1.append(x_1)
        y_1 = y[i]
        Y1.append(y_1)
        z_1 = z[i]
        Z1.append(z_1)
        x_2 = x1[i]
        X2.append(x_2)
        y_2 = y1[i]
        Y2.append(y_2) 
        z_2 = z1[i]
        Z2.append(z_2)
    
    return X1,Y1,Z1,X2,Y2,Z2
