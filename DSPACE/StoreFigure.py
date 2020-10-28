import plotly.graph_objects as go
import plotly
import pandas as pd
from numpy import *
import csv
fig1 = go.Figure()
l1 = []
l2 = []
for i in range(10):
    fig = go.Figure()
    data = go.Scatter3d(x=[i],y=[i+1],z =[i+2])
    fig.add_trace(data)
    
    l1.append(fig.data)
l2.append(l1)
print(l2[0][1])
df=pd.DataFrame(data=l2)
df.to_csv('data.csv',index=False,header = False)
datafig = pd.read_csv('data.csv')
print(datafig)
datafig = list(datafig)
for i in range(len(datafig)):
    datafig[i] = tuple(datafig[i])

print(type(datafig[1]))
print(type(l2[0][1]))
# datafig.astype(tuple)
# l3 = list(fig1.data)
# l3.append(datafig[1])
fig1.data = datafig[1]
fig1.show()