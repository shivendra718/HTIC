import urx
import math
from math import *
import math3d as m3d
import numpy as np
import time
import plotly.graph_objects as go
import pandas as pd
from dspace import *



datadspace = pd.read_csv('data.csv')
fig = RobotBaseStructure()
xptsd = list(datadspace['xptsd'])
yptsd = list(datadspace['yptsd'])
zptsd = list(datadspace['zptsd'])
datapts = pd.read_csv('data1.csv')
xpts = list(datapts['xpts'])
ypts = list(datapts['ypts'])
zpts = list(datapts['zpts'])
print(xptsd,yptsd,zptsd)

data1 = go.Scatter3d(x=xpts,y=ypts,z=zpts,mode = 'markers', marker=dict(opacity = 0.5, color = ('rgb(204, 102, 0)')))
data2 = go.Scatter3d(x=xptsd,y=yptsd,z=zptsd,mode = 'markers',marker=dict(opacity = 0.5))
data3 = go.Mesh3d(x=xptsd,y=yptsd,z=zptsd,alphahull = 3,opacity = 0.5)
# data3 = go.Surface(x=xptsd,y=yptsd,z=zptsd)

fig.add_trace(data1)
fig.show()
fig.add_trace(data2)
fig.show()
# fig.add_trace(data3)
fig.data = traceremoval(fig,data1)
fig.show()