import numpy as np
from modules import *
import plotly.graph_objects as go
from numpy import *
from CollisionFunction import ComputeTransformation


height= 0 

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

# def plane(fig,height):
#     x= np.linspace(-800, -500, 75)
#     y= np.linspace(800, -800, 100)
#     z= height*np.ones((5000,200))
#     mycolorscale = [[0, '#aa9ce2'],
#                 [1, '#aa9ce2']]
#     surf = go.Surface(x=x,y=y,z=z,colorscale = mycolorscale, showscale=False)
#     fig.add_trace(surf)
#     return fig

def plane(fig,p1,p2,p3,p4):
    x = [p1[0],p2[0],p3[0],p4[0]]
    y = [p1[1],p2[1],p3[1],p4[1]]
    z = [p1[2],p2[2],p3[2],p4[2]]

    PlaneData = {
    'type': 'mesh3d',        
    'x': x,
    'y': y,
    'z': z, 
    #'delaunayaxis':'x',
    'color': 'green',
    'opacity': 0.5,
    }
    fig.add_trace(PlaneData)

    return fig






#Robot base cylinder    
def RobotBaseCylinder(r, h, a =0, nt=100, nv =50):
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = 0+r*np.cos(theta)
    y= 0+r*np.sin(theta)
    z = 0+v
    return x, y, z

def RobotBaseStructure():
    
    thetas = [0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
    temp_joint_locations = thetas
    sl = JointLocations(temp_joint_locations)
    x_p,y_p,z_p = SepPoints(sl)

    x_p.insert(1,0)
    x_p.insert(2,-2.602374e-17)

    y_p.insert(1,-0.1333)
    y_p.insert(2,-0.1333)

    z_p.insert(1,0.1625)
    z_p.insert(2,0.5875)    


    x_p=list(np.asarray(x_p) * 1000)
    y_p=list(np.asarray(y_p) * 1000)
    z_p=list(np.asarray(z_p) * 1000)

    data=go.Scatter3d(x=x_p,
                        y=y_p,
                        z=z_p,marker=dict(
                size=[40,40,36,36,28,28,28,28],
                opacity = 0,
                color = ('rgb(22, 96, 167)'),              
                colorscale='Viridis',  

            ),
                line = dict(
                colorscale="Viridis",
                width = 50,
                ),
            )
    fig = go.Figure(data = data)
    fig.update_layout(scene=dict(zaxis=dict(range=[-925,925],autorange=False),       
                        yaxis=dict(range=[-925,925],autorange=False),
                    xaxis=dict(range=[-925,925],autorange=False)))               
    fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

    fig.add_trace(data)
    Data = go.Mesh3d(
        
        x=[400, 400, 0, 0, 400, 400, 0, 0],
        y=[400, -450, -450, 400, 400, -450, -450, 400],
        z=[0, 0, 0, 0, -700, -700, -700, -700],
        colorbar_title='z',
       
        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        name='y',
        showscale=True
    )
    fig.add_trace(Data)
    
    #Base cylinder
    r1 = 45
    a1 = 0
    h1 = 150
    x1, y1, z1 = RobotBaseCylinder(r1, h1, a=a1)
    colorscale = [[0, 'red'],
                    [1, 'red']]
    cyl1 = go.Surface(x=x1, y=y1, z=z1,
                    colorscale=colorscale,
                    showscale=False,)
    #############auto -925 925

    fig.add_trace(cyl1)

    #Plane data:
    height = 0
    Pp1 = [-800,-750,height]
    Pp2 = [-250,-750,height]
    Pp3 = [-800,750,height]
    Pp4 = [-250,750,height]

    fig = plane(fig,Pp1,Pp2,Pp3,Pp4)

    return fig


def TracePath(fig,xp,yp,zp):

    Pathdata = go.Scatter3d(x=xp, y=yp, z=zp,
                             marker=dict(
            size=6,
    #         color=z,                
            color = ('rgb(22, 96, 167)'),   
            opacity=0.9
        ),line = dict(width=4))
    
    fig.add_trace(Pathdata)
    return fig,Pathdata

def TraceEntTarpoints(fig,Ep,Tp):

    EpTpPoints = go.Scatter3d(x=[Ep[0],Tp[0]], y=[Ep[1],Tp[1]], z=[Ep[2],Tp[2]],
                            line = dict(width=4),
                            marker=dict(
            size = 12,
    #         color=z,                
            color = ('rgb(78, 12, 11)'),   
            opacity=0.9
        ))
    
    fig.add_trace(EpTpPoints)
    return fig,EpTpPoints

def plot_frames(fig,xps,yps,zps):
    
    fig.update_layout(width=1800,
                scene_camera_eye_z=0.75,
                title="Start Title",
            updatemenus=[dict(
            type="buttons",
            buttons=[dict(label="Play",
                        method="animate",
                        args=[None])])])
    
    for i in range(len(xps)):
        b=list(fig.frames)
        b.append(go.Frame(data=go.Scatter3d(x = np.asarray(xps[i])*1000,
                                            y = np.asarray(yps[i])*1000,
                                            z = np.asarray(zps[i])*1000,
                                            marker=dict(
                size=[40,40,36,36,28,28,28,28],              
                colorscale='Viridis',
                color = ('rgb(0, 0, 0)'),
                opacity = 0         

            ),
            line = dict(
                colorscale="Viridis",
                width = 50,),)))
        fig.frames = tuple(b)
    # fig.add_trace(data)

    return fig



def DisplayEllipsoid(fig,c,TMat,r1,r2):
    
    theta = linspace(0,2*np.pi,100)
    phi = linspace(0,np.pi,100)
    x =  (r1) * outer(cos(theta),sin(phi))
    y =  (r2) * outer(sin(theta),sin(phi))
    z =  (r2) * outer(ones(100),cos(phi))
    x1,y1,z1 = ComputeTransformation(x.flatten(),y.flatten(),z.flatten(),TMat,1)

    x1 = c[0] + x1
    y1 = c[1] + y1
    z1 = c[2] + z1

    EllipsoidData=go.Surface(
    x=x1,
    y=y1,
    z=z1,
    opacity=0.8)

    fig.add_trace(EllipsoidData)

    return fig


def DeleteGraphPoints(Plot_Figure,Pathdata,EpTpPoints):
    Current_data = list(Plot_Figure.data)
    Current_data.remove(Pathdata)
    Current_data.remove(EpTpPoints)
    Plot_Figure.frames = ()
    Plot_Figure.data = tuple(Current_data)

    return Plot_Figure