from scipy.linalg import norm
import numpy as np
import plotly.graph_objects as go

p1 = [4,1,4]
p2 = [1,1,4]
p3 = [1,4,4]
p4 = [4,4,4]


vector1 = np.subtract(p4,p2)
vector2 = np.subtract(p3,p1)
mag1 = norm(vector1)
mag2 = norm(vector2)
unit_v1 = [vector1[0]/mag1,vector1[1]/mag1,vector1[2]/mag1]
unit_v2 = [vector2[0]/mag2,vector2[1]/mag2,vector2[2]/mag2]

unit_v1 = np.asarray(unit_v1)
unit_v2 = np.asarray(unit_v2)

t = 2

new_vector1 = (t/2 + mag1)*unit_v1
new_vector2 = (t/2 + mag2)*unit_v2

new_p1 = np.subtract(p3,new_vector1)
new_p3 = np.add(new_vector1,p1)
new_p2 = np.subtract(p4,new_vector2)
new_p4 = np.add(new_vector2,p2)

# print(new_p1,new_p2,new_p3,new_p4)


#fig.show()

data = [{
    'type': 'mesh3d',        
    'x': [new_p1[0],new_p2[0],new_p3[0],new_p4[0]],
    'y': [new_p1[1],new_p2[1],new_p3[1],new_p4[1]],
    'z': [new_p1[2],new_p2[2],new_p3[2],new_p4[2]], 
    #'delaunayaxis':'x',
    'color': 'green',
    'opacity': 0.5,
}]

fig = go.Figure(data = data)
# fig.show()

pt1 = [2,6,3]
pt2 = [4,2,5]

data1 = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]],
                    mode="lines+markers+text",
                    name="Markers and Text",
                    text=["Text 1", "Text 2"],
                    textposition="bottom center")
fig.add_trace(data1)
# fig.show()
vector = np.subtract(pt2,pt1)



# #Define plane
planeNormal = np.array([0, 0, 1])
planePoint = np.array([7, 5.65, 4]) #Any point on the plane


w = np.subtract(pt1,planePoint)

N = -np.dot(planeNormal,w)
D = np.dot(planeNormal,vector)
sI = N/D
I=pt1+(sI*vector)
print(I)

## for removing traces
# print(fig.data)
# data2 = list(fig.data)
# data2.remove(data1)
# fig.data = tuple(data2)
fig.show()