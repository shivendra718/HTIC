from scipy.linalg import norm
import numpy as np
import plotly.graph_objects as go
fig = go.Figure()
def plane(p1,p2,p3,p4):
    x = [p1[0],p2[0],p3[0],p4[0]]
    y = [p1[1],p2[1],p3[1],p4[1]]
    z = [p1[2],p2[2],p3[2],p4[2]]
    

    data = {
    'type': 'mesh3d',        
    'x': x,
    'y': y,
    'z': z, 
    #'delaunayaxis':'x',
    'color': 'green',
    'opacity': 0.5,
    }
    fig.add_trace(data)
    
    # x = x[1]
    # y = y[1]
    # z = z[1]
    return fig,x,y,z


def collision_plane(pt1,pt2,eps = 1e-6):
    xmin = min(x[0],x[1],x[2],x[3])
    xmax = max(x[0],x[1],x[2],x[3])
    ymin = min(y[0],y[1],y[2],y[3])
    ymax = max(y[0],y[1],y[2],y[3])

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
    planePoint = np.array([x[1],y[1],z[1]])    #Any point on the plane
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
            
    # if ((pt1[0] <= xmax and pt1[0] >= xmin) and (pt1[1] <= ymax and pt1[1] >= ymin)) or ((pt2[0] <= xmax and pt2[0] >= xmin) and (pt2[1] <= ymax and pt2[1] >= ymin)):
    if (I1[0] <= xmax and I1[0] >= xmin) and (I1[1] <= ymax and I1[1] >= ymin) and I1[2] == z[1]:
        if abs(D) > eps:
            dist1 = norm(np.subtract(pt1,I1))
            dist2 = norm(np.subtract(pt2,I1))
                
            if (threshold < dist1 or threshold < dist2):
                distance.append(min(dist1,dist2))
                print("not intersecting")
            elif (threshold > dist1 or threshold > dist2):
                distance.append(0)
                print("interecting")

    else:
        if pt1[2] == pt2[2] == z[1]:
            distance.append(0)
            print("parallel but inside plane")
        elif pt1[2] == pt2[2] != z[1]:
            distance.append(-1)
            print("parallel")
        else:
            distance.append(-1)
            print("not interseccting")
        


    print(distance)
    return fig
        

fig,x,y,z = plane([6,3,6],[3,3,6],[3,6,6],[6,6,6])
fig = collision_plane([5,6,8],[5,5,8])
fig = collision_plane([5,4,6],[5,4.5,6])
# fig.update_layout(scene=dict(zaxis=dict(range=[0,50],autorange=False),       
#                        yaxis=dict(range=[0, 50],autorange=False),
#                    xaxis=dict(range=[0,50],autorange=False)))   
fig.show()

