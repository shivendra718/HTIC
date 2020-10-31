from scipy.linalg import norm
import numpy as np
import plotly.graph_objects as go
fig = go.Figure()
def plane(h):
    x= np.linspace(-1, 100, 75)
    y= np.linspace(0, 100, 100)
    z= h*np.ones((100,75))

    mycolorscale = [[0, '#aa9ce2'],
                [1, '#aa9ce2']]

    surf = go.Surface(x=x, y=y, z=z, colorscale=mycolorscale, showscale=False)
    fig.add_trace(surf)
    x = list(x)
    y = list(y)
    z = list(z)
    x = x[50]
    y = y[50]
    z = z[50][1] 
    return fig,x,y,z


def collision_plane(pt1,pt2,eps = 1e-6):

    data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]])
    fig.add_trace(data)
    vector = np.subtract(pt2,pt1)
    mag = norm(vector)
    threshold = mag
    unit_v = [vector[0]/mag,vector[1]/mag,vector[2]/mag]
    unit_v = np.asarray(unit_v)
    new_pt1 = pt1 + (mag*vector)
    new_pt2 = (pt1 - (mag*vector))
    vector1 = np.subtract(new_pt2,new_pt1)
    # data = go.Scatter3d(x=[new_pt1[0],new_pt2[0]], y=[new_pt1[1],new_pt2[1]], z=[new_pt1[2],new_pt2[2]])
    # fig.add_trace(data)
    




# #Define plane
    planeNormal = np.array([0, 0, 1])
    planePoint = np.array([x,y,z])    #Any point on the plane
    distance = []

    w = np.subtract(pt1,planePoint)
    N = -np.dot(planeNormal,w)
    D = np.dot(planeNormal,unit_v)
    sI = N/D
    I=pt1+(sI*unit_v)


    w1 = np.subtract(new_pt1,planePoint)                   
    N1 = -np.dot(planeNormal,w1)
    D1 = np.dot(planeNormal,vector1)
    sI1 = N1/D1                                                                              
    I1=new_pt1+(sI1*vector1)

    if abs(D) > eps:
        dist1 = norm(np.subtract(pt1,I1))
        dist2 = norm(np.subtract(pt2,I1))
    
        if (threshold < dist1 or threshold < dist2):
            distance.append(min(dist1,dist2))
        elif (threshold > dist1 or threshold > dist2):
            distance.append(0)
    else :
        distance.append(-1)



    print(distance)
    return fig

fig,x,y,z = plane(4)
fig = collision_plane([40,40,5],[50,50,5])
# fig.update_layout(scene=dict(zaxis=dict(range=[0,50],autorange=False),       
#                        yaxis=dict(range=[0, 50],autorange=False),
#                    xaxis=dict(range=[0,50],autorange=False)))   
fig.show()

