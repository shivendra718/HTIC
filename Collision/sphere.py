import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm

def sphere(c,r):
    fig = go.Figure()
    
    theta = linspace(0,2*np.pi,100)
    phi = linspace(0,np.pi,100)

    x = c[0]+r*outer(cos(theta),sin(phi))
    y = c[1]+r*outer(sin(theta),sin(phi))
    z = c[2]+r*outer(ones(100),cos(phi))

    data=go.Surface(
        x=x,
        y=y,
        z=z,
        opacity=0.3
    )


    


    fig.add_trace(data)
    return fig

fig = sphere([1,2,3],1.6)
fig.show()