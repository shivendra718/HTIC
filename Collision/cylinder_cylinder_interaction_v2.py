import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm

# global variables for cylinder and collision_line functions.
start_end = []
radius = []
step_ = []


def plane(height):
    
    x= np.linspace(-800, -500, 75)
    y= np.linspace(800, -800, 100)
    z= height*np.ones((5000,200))
    mycolorscale = [[0, '#aa9ce2'],
                [1, '#aa9ce2']]
    surf = go.Surface(x=x,y=y,z=z,colorscale = mycolorscale, showscale=False)
    fig.add_trace(surf)
    return fig
    
def JointLocations(thetas):
   
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
   
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
   
    theta1 = [0,0,d1]
   
    theta2 = [(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]
               
    theta3 = [np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]
               
    theta4 = [(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]
               
    theta5 = [ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))]
               
    theta6 = [((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))]
   
    positions = [theta1,theta2,theta3,theta4,theta5,theta6]
    return positions

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

def cylinder(r, h, a =0, nt=100, nv =50):
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = (r*np.cos(theta))
    y= (r*np.sin(theta))
    z = v
    return x, y, z


def Cylinder(pt1,pt2,r):
    data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]],
                        line = dict(width=50))
    fig.add_trace(data)
    
    v = subtract(pt2,pt1)
    mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
    unit_v=asarray(unit_v)
    dist = sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2 + (pt2[2]-pt1[2])**2)
    v1 = pt1
    v2 = pt2
    pdist = dist/step
    newpts_array = []
    

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
    

    start_end.append([v1,v2])
    radius.append(r)
    step_.append(step)
    return fig

def robot_movement(thetaS,thetaE,frames_):
    theta = []
    for i in range(0,len(thetaS)):
        b = thetaE[i]-thetaS[i]
        theta.append(b)

    temp_joint_locations = thetaS
    sl = JointLocations(temp_joint_locations)
    joint_coordinates = []
    coordinates = []
    inc_=np.divide(theta,frames_)
    x_move=[]
    y_move=[]
    z_move=[]

    x_move1 = []
    y_move1 = []
    z_move1 = []

    # v0 = [0,0,0.1625]
    # v1 = [0,-0.1333,0.1625]                # This commented portion is for the extra 2 joints that are being added to the 6 joints given.
    # V = np.subtract(v1,v0)

    # v3 = [-2.602374e-17,0,0.5875]
    # axis = np.subtract(v3,v0)
    # axis_mag = np.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    # axis_unit = (axis[0]/axis_mag,axis[1]/axis_mag,axis[2]/axis_mag)
    # axis_unit = np.asarray(axis_unit)
    # h=[0.,0.,0.]
    # h=np.asarray(h)
    # x_points = []
    # y_points = []
    # z_points = []  
    # for i in range(0,frames_):
    #     h += np.asarray(V*np.cos(inc_[0])+np.cross(axis_unit,V)*np.sin(inc_[0])+ axis_unit*(np.dot(axis_unit,V))*(1-np.cos(inc_[0])))
    #     x_points.append(h[0])
    #     y_points.append(h[1])
    #     z_points.append(h[2])

    for j in range(frames_):
        temp_joint_locations = np.add(temp_joint_locations,inc_)
        joint_coordinates.append(np.array(temp_joint_locations)) 
        
    for j in range(len(joint_coordinates)):
        temp_values = JointLocations(joint_coordinates[j])
        coordinates.append(np.array(temp_values))
    for j in range(len(coordinates)):
        x_,y_,z_ = SepPoints(coordinates[j])
        d = 0.1333
        temp = 0

        temp=temp - d

        x_.insert(1,x_[0])
        y_.insert(1,temp)
        z_.insert(1,z_[0])

        x_.insert(2,x_[2])
        y_.insert(2,temp)
        z_.insert(2,z_[2])
        
        x_=np.array(x_)*1000
        y_=np.array(y_)*1000
        z_=np.array(z_)*1000
        x_=list(x_)
        y_=list(y_)
        z_=list(z_)
        #print(x_)

        x_move.append(x_)
        y_move.append(y_)
        z_move.append(z_)
        # print(x_move)


        for i in range(len(x_)-1):
            pt1 = [x_[i],y_[i],z_[i]]
            pt2 = [x_[i+1],y_[i+1],z_[i+1]]
            fig = collision_line(pt1,pt2)
            v = np.subtract(pt2,pt1)
            mag = norm(v)
            unit_v = [v[0]/mag,v[1]/mag,v[2]/mag]
            unit_v = asarray(unit_v)
            p = pt1 + (mag*v)
            p_ = -(pt1 + (mag*v))
            x_move1.append(p[0])
            y_move1.append(p[1])
            z_move1.append(p[2])
            
            data = go.Scatter3d(x=[pt1[0],p[0]], y=[pt1[1],p[1]], z=[pt1[2],p[2]],marker = dict(opacity = 0.99))
            fig.add_trace(data)
            data = go.Scatter3d(x=[pt1[0],p_[0]], y=[pt1[1],p_[1]], z=[pt1[2],p_[2]],marker = dict(opacity = 0.99))
            fig.add_trace(data)
        # for i in range(len(x_move)-1):
        #     for j in range(frames_):
        #         pt1 = [x_move[i][j],y_move[i][j],z_move[i][j]]
        #         pt2 = [x_move[i][j+1],y_move[i][j+1],z_move[i][j+1]]
        #         fig = collision_line(pt1,pt2)
        #         v = np.subtract(pt2,pt1)
        #         mag = norm(v)
        #         unit_v = [v[0]/mag,v[1]/mag,v[2]/mag]
        #         unit_v = asarray(unit_v)
        #         p = pt1 + (mag*v)
        #         p_ = -(pt1 + (mag*v))
        #         x_move1.append(p[0])
        #         y_move1.append(p[1])
        #         z_move1.append(p[2])
                
        #         data = go.Scatter3d(x=[pt1[0],p[0]], y=[pt1[1],p[1]], z=[pt1[2],p[2]],marker = dict(opacity = 0.99))
        #         fig.add_trace(data)
        #         data = go.Scatter3d(x=[pt1[0],p_[0]], y=[pt1[1],p_[1]], z=[pt1[2],p_[2]],marker = dict(opacity = 0.99))
        #         fig.add_trace(data)

        print("\n")
        
    

        

    fig.update_layout(width=1800,
                  scene_camera_eye_z=0.75,
                  title="Start Title",
                updatemenus=[dict(
                type="buttons",
                buttons=[dict(label="Play",
                          method="animate",
                          args=[None])])])
    
    for i in range(frames_):
        b=list(fig.frames)
        b.append(go.Frame(data=go.Scatter3d(x=x_move[i],
                                            y=y_move[i],
                                            z=z_move[i],
                                            marker=dict(
                size=[40,40,36,36,28,28,28,28],              
                colorscale='Viridis',
                color = ('rgb(0, 0, 0)'),
                opacity = 0         

            ),
            line = dict(
                colorscale="Viridis",
                width = 50,),
                
                                                )))

        # b.append(go.Frame(data=go.Scatter3d(x=[x_move1[i]],
        #                                     y=[y_move1[i]],
        #                                     z=[z_move1[i]],
        #                                     marker=dict(opacity= 1)
            
                
        #                                         )))
                                            
          
    
        
        fig.frames=tuple(b)
    fig.add_trace(data)
    
       

    Data=go.Mesh3d(
        
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
    return fig