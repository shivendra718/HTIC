# This code represents the robot visualization ie., the movement of the robot in the given environment with fixed axis.

import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm
from cylinder_line_v1 import *

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
    step = 7
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
    for i in range(0, step):
        dist_v = pdist*unit_v
        newpts = pt1 + dist_v
        newpts_array.append(newpts)
        pt1 = newpts
        newpts=list(newpts)
        theta = linspace(0,2*np.pi,100)
        phi = linspace(0,np.pi,100)

        x = newpts[0]+r*outer(cos(theta),sin(phi))
        y = newpts[1]+r*outer(sin(theta),sin(phi))    
        z = newpts[2]+r*outer(ones(100),cos(phi))

        data=go.Surface(
            x=x,
            y=y,
            z=z,
            opacity=0.3
        )
        #fig.add_trace(data)

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

    
def collision_line(p1,p2):
    # data = go.Scatter3d(x=[p1[0],p2[0]], y=[p1[1],p2[1]], z=[p1[2],p2[2]])
    # fig.add_trace(data)
    # newpts_opp = []
    v_line = np.subtract(p2,p1)
    mag1 = norm(v_line)
    newpts_ =  p1 + (mag1*v_line)
    newpts_opp = -(p1 + (mag1*v_line))
    data = go.Scatter3d(x=[p1[0],newpts_[0]], y=[p1[1],newpts_[1]], z=[p1[2],newpts_[2]])
    fig.add_trace(data)
    data = go.Scatter3d(x=[p1[0],newpts_opp[0]], y=[p1[1],newpts_opp[1]], z=[p1[2],newpts_opp[2]])
    fig.add_trace(data)


    # print(newpts_)
    # print(newpts_opp)
    v_list = []
    distance = []
    for i in range(len(start_end)):
        newpts_array = []
        v = np.subtract(start_end[i][1],start_end[i][0])
        v_list.append(v)
        mag = norm(v)
        unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
        unit_v=asarray(unit_v)
        pt1 = start_end[i][0]
        pt2 = start_end[i][1]
        r = radius[i]
        step = step_[i]
        pdist = mag/step
        dist_array_ni = []
        dist_array = []
        dist_array_lni = []
        

        for j in range(0,step):
            dist_v = pdist*unit_v
            newpts = pt1 + dist_v
            newpts_array.append(newpts)
            pt1 = newpts
            newpts=list(newpts)

            a1 = (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2
            b1 = 2*((p2[0]-p1[0])*(p1[0]-newpts_array[j][0]) + (p2[1]-p1[1])*(p1[1]-newpts_array[j][1]) + (p2[2]-p1[2])*(p1[2]-newpts_array[j][2]))
            c1 = (newpts_array[j][0]**2 + newpts_array[j][1]**2 + newpts_array[j][2]**2 + p1[0]**2 + p1[1]**2 + p1[2]**2 - 2*(newpts_array[j][0]*p1[0] + 
                                                                        newpts_array[j][1]*p1[1] + newpts_array[j][2]*p1[2]) - r**2)

            a2 = (newpts_[0]-p1[0])**2 + (newpts_[1]-p1[1])**2 + (newpts_[2]-p1[2])**2
            b2 = 2*((newpts_[0]-p1[0])*(p1[0]-newpts_array[j][0]) + (newpts_[1]-p1[1])*(p1[1]-newpts_array[j][1]) + (newpts_[2]-p1[2])*(p1[2]-newpts_array[j][2]))
            c2 = (newpts_array[j][0]**2 + newpts_array[j][1]**2 + newpts_array[j][2]**2 + p1[0]**2 + p1[1]**2 + p1[2]**2 - 2*(newpts_array[j][0]*p1[0] + 
                                                                        newpts_array[j][1]*p1[1] + newpts_array[j][2]*p1[2]) - r**2)
        
            discriminant2 = (b2**2 - (4 * a2 * c2))
            


        
            discriminant1 = (b1**2 - (4 * a1 * c1))
            
            
            if discriminant1 >= 0:
                x_1=(-b1+math.sqrt(discriminant1))/(2*a1)
                x_2=(-b1-math.sqrt(discriminant1))/(2*a1)
            else:
                x_1= complex((-b1/(2*a1)),math.sqrt(-discriminant1)/(2*a1))
                x_2= complex((-b1/(2*a1)),-math.sqrt(-discriminant1)/(2*a1))

            sol1 = [p1[0]*(1-x_2) + x_2*p2[0],
                p1[1]*(1-x_2) + x_2*p2[1],
                p1[2]*(1-x_2) + x_2*p2[2]]              

            sol2 = [p1[0]*(1-x_1) + x_1*p2[0],
                p1[1]*(1-x_1) + x_1*p2[1],
                p1[2]*(1-x_1) + x_1*p2[2]]
            

            if discriminant2 >= 0:
                x_12=(-b2+math.sqrt(discriminant2))/(2*a2)
                x_22=(-b2-math.sqrt(discriminant2))/(2*a2)
            else:
                x_12= complex((-b2/(2*a2)),math.sqrt(-discriminant2)/(2*a2))
                x_22= complex((-b2/(2*a2)),-math.sqrt(-discriminant2)/(2*a2))

            sol11 = [p1[0]*(1-x_22) + x_22*newpts_[0],
                p1[1]*(1-x_22) + x_22*newpts_[1],
                p1[2]*(1-x_22) + x_22*newpts_[2]]              

            sol22 = [p1[0]*(1-x_12) + x_12*newpts_[0],
                p1[1]*(1-x_12) + x_12*newpts_[1],
                p1[2]*(1-x_12) + x_12*newpts_[2]]

            
           

            
            if (discriminant1 < 0):
                dist1 = sqrt((newpts[0]-p1[0])**2 + (newpts[1]-p1[1])**2 + (newpts[2]-p1[2])**2) - r
                dist2 = sqrt((newpts[0]-p2[0])**2 + (newpts[1]-p2[1])**2 + (newpts[2]-p2[2])**2) - r
                if dist1<dist2:
                    dist1_ = dist1
                    dist_array_ni.append(dist1_)
                else:
                    dist2_ = dist2
                    dist_array_ni.append(dist2_)
                
                
            elif((x_2>1 or x_2<0) and (x_1>1 or x_1<0)):
                dist1 = sqrt((newpts[0]-p1[0])**2 + (newpts[1]-p1[1])**2 + (newpts[2]-p1[2])**2) - r
                dist2 = sqrt((newpts[0]-p2[0])**2 + (newpts[1]-p2[1])**2 + (newpts[2]-p2[2])**2) - r
                if dist1<dist2:
                    dist3_ = dist1
                    dist_array_ni.append(dist3_)
                else:
                    dist4_ = dist2
                    dist_array_ni.append(dist4_)
            
            elif (not(x_2 > 1 or x_2 < 0) and (x_1 > 1 or x_1 < 0)):
                dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
                dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
                if dist1<dist2:
                    dist5_ = dist1
                    dist_array.append(dist5_)
                else: 
                    dist6_ = dist2
                    dist_array.append(dist6_)
            elif ((x_2 > 1 or x_2 < 0) and not(x_1 > 1 or x_1 < 0)):
                dist1 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
                dist2 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
                if dist1<dist2:
                    dist7_ = dist1
                    dist_array.append(dist7_)
                else: 
                    dist8_ = dist2
                    dist_array.append(dist8_)
            elif (not(x_2 > 1 or x_2 < 0) and not(x_1 > 1 or x_1 < 0)):
                dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
                dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
                dist3 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
                dist4 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
                if (dist1<dist2 and dist1<dist3 and dist1<dist4):
                    dist9_ = dist1
                    dist_array.append(dist9_)
                elif (dist2<dist1 and dist2<dist3 and dist2<dist4):
                    dist10_ = dist2
                    dist_array.append(dist10_)
                elif (dist3<dist1 and dist3<dist2 and dist3<dist4):
                    dist11_ = dist3
                    dist_array.append(dist11_)
                elif (dist4<dist1 and dist4<dist2 and dist4<dist3):
                    dist12_ = dist4
                    dist_array.append(dist12_)

            if (discriminant2 < 0):
                dist1 = sqrt((newpts[0]-p1[0])**2 + (newpts[1]-p1[1])**2 + (newpts[2]-p1[2])**2) - r
                dist2 = sqrt((newpts[0]-p2[0])**2 + (newpts[1]-p2[1])**2 + (newpts[2]-p2[2])**2) - r
                if dist1<dist2:
                    dist1_ = dist1
                else:
                    dist2_ = dist2
                    

            elif (not(x_22 > 1 or x_22 < 0) and (x_12 > 1 or x_12 < 0)):
                dist1 = sqrt((sol11[0]-p1[0])**2 + (sol11[1]-p1[1])**2 + (sol11[2]-p1[2])**2)
                dist2 = sqrt((sol11[0]-p2[0])**2 + (sol11[1]-p2[1])**2 + (sol11[2]-p2[2])**2)
                print(dist1,dist2)
                if dist1<dist2:
                    dist5_ = dist1
                    dist_array_lni.append(dist5_)
                else: 
                    dist6_ = dist2
                    dist_array_lni.append(dist6_)
            elif ((x_22 > 1 or x_22 < 0) and not(x_12 > 12 or x_12 < 0)):
                dist1 = sqrt((sol22[0]-p1[0])**2 + (sol22[1]-p1[1])**2 + (sol22[2]-p1[2])**2)
                dist2 = sqrt((sol22[0]-p2[0])**2 + (sol22[1]-p2[1])**2 + (sol22[2]-p2[2])**2)
                print(dist1,dist2)
                if dist1<dist2:
                    dist7_ = dist1
                    dist_array_lni.append(dist7_)
                else: 
                    dist8_ = dist2
                    dist_array_lni.append(dist8_)
            else:
                dist1 = sqrt((sol11[0]-p1[0])**2 + (sol11[1]-p1[1])**2 + (sol11[2]-p1[2])**2)
                dist2 = sqrt((sol11[0]-p2[0])**2 + (sol11[1]-p2[1])**2 + (sol11[2]-p2[2])**2)
                dist3 = sqrt((sol22[0]-p1[0])**2 + (sol22[1]-p1[1])**2 + (sol22[2]-p1[2])**2)
                dist4 = sqrt((sol22[0]-p2[0])**2 + (sol22[1]-p2[1])**2 + (sol22[2]-p2[2])**2)
                if (dist1<dist2 and dist1<dist3 and dist1<dist4):
                    dist9_ = dist1
                    dist_array.append(dist9_)
                elif (dist2<dist1 and dist2<dist3 and dist2<dist4):
                    dist10_ = dist2
                    dist_array_lni.append(dist10_)
                elif (dist3<dist1 and dist3<dist2 and dist3<dist4):
                    dist11_ = dist3
                    dist_array_lni.append(dist11_)
                elif (dist4<dist1 and dist4<dist2 and dist4<dist3):
                    dist12_ = dist4
                    dist_array_lni.append(dist12_)
        dist_array_ni.sort()
        dist_array_lni.sort()
        # print(dist_array)
        # print(dist_array_ni)
        # print(dist_array_lni)



        
        

        if (dist_array == [] and dist_array_ni != [] and dist_array_lni == []):
            d_ni = -1
            distance.append(d_ni)
        elif (dist_array == [] and dist_array_ni != [] and dist_array_lni != []):
            distance.append(dist_array_lni[0])
                
        else:
            d_inter = 0
            distance.append(d_inter)
    print(distance) 
    
    return fig



#def display_path():


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
        

        x_move.append(x_)
        y_move.append(y_)
        z_move.append(z_)
        print(len(x_))


        # for i in range(len(x_)-1):
        #     pt1 = [x_[i],y_[i],z_[i]]
        #     pt2 = [x_[i+1],y_[i+1],z_[i+1]]
        #     fig = collision_line(pt1,pt2)
        #     v = np.subtract(pt2,pt1)
        #     mag = norm(v)
        #     unit_v = [v[0]/mag,v[1]/mag,v[2]/mag]
        #     unit_v = asarray(unit_v)
        #     p = pt1 + (mag*v)
        #     p_ = -(pt1 + (mag*v))
        #     x_move1.append(p[0])
        #     y_move1.append(p[1])
        #     z_move1.append(p[2])
            
        #     data = go.Scatter3d(x=[pt1[0],p[0]], y=[pt1[1],p[1]], z=[pt1[2],p[2]],marker = dict(opacity = 0.99))
        #     fig.add_trace(data)
        #     data = go.Scatter3d(x=[pt1[0],p_[0]], y=[pt1[1],p_[1]], z=[pt1[2],p_[2]],marker = dict(opacity = 0.99))
        #     fig.add_trace(data)

        # print("\n")
        
    

        

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
        
        # for j in range (9):
        #     b.append(go.Frame(data=cylinder_line(fig,[x_move[i][j],y_move[i][j],z_move[i][j]],[x_move[i][j+1],y_move[i][j+1],z_move[i][j+1]],10).data))

        # for j in range(len(x_)-1):
        #     fig = cylinder_line(fig,[x_move[i],y_move[i],z_move[i]],[x_move[i+1],y_move[i+1],z_move[i+1]],10)
        #     b.append(b)

    
        
        fig.frames=tuple(b)
        # print(fig.frames)
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
    return fig,x_,y_,z_
   
    
#### main 
data=go.Mesh3d(
        
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

fig = go.Figure(data = data)
fig.update_layout(scene=dict(zaxis=dict(range=[-925,925],autorange=False),       
                       yaxis=dict(range=[-925, 925],autorange=False),
                   xaxis=dict(range=[-925,925],autorange=False)))               
fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

r1 = 5
a1 = 0
h1 = 150
x1, y1, z1 = cylinder(r1, h1, a=a1)
colorscale = [[0, 'red'],
                [1, 'red']]
cyl1 = go.Surface(x=x1, y=y1, z=z1,
                 colorscale=colorscale,
                 showscale=False,)


fig.add_trace(cyl1)


thetas=[0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
thetae=[0.3803219795227051,-0.6225808423808594,1.3531478087054651,-0.6644571584514161,0.43364858627319336,3.082611560821533]  #theta1 rotation compensation

temp_joint_locations=thetas
sl = JointLocations(temp_joint_locations)

x_p,y_p,z_p = SepPoints(sl)


x_p.insert(1,0)
x_p.insert(2,-2.602374e-17)

y_p.insert(1,-0.1333)
y_p.insert(2,-0.1333)

z_p.insert(1,0.1625)
z_p.insert(2,0.5875)

x_p=np.array(x_p)*1000 #####1000
y_p=np.array(y_p)*1000
z_p=np.array(z_p)*1000
x_p=list(x_p)
y_p=list(y_p)
z_p=list(z_p)
print(x_p)
# for i in range(len(x_)-1):
#     X1,Y1,Z1,X2,Y2,Z2 = cylinder_line([x_[i],y_[i],z_[i]],[x_[i+1],y_[i+1],z_[i+1]],10)
#     for j in range(len(X1)):
#         d = go.Scatter3d(x=[X1[j],X2[j]],y=[Y1[j],Y2[j]],z=[Z1[j],Z2[j]])
#         fig.add_trace(d)
    # print(fig)
# fig = cylinder_line(fig,[x_p[7],y_p[7],z_p[7]],[x_p[6],y_p[6],z_p[6]],2)
# fig = cylinder_line(fig,[x_p[4],y_p[4],z_p[4]],[x_p[3],y_p[3],z_p[3]],2)

# data=go.Scatter3d(x=x_p,
#                        y=y_p,
#                        z=z_p,marker=dict(
#             size=[40,40,36,36,28,28,28,28],
#             opacity = 0,
#             color = ('rgb(22, 96, 167)'),              
#             colorscale='Viridis',  

#         ),
#             line = dict(
#             colorscale="Viridis",
#             width = 50,
#             ),
#         )
fig.add_trace(data)
fig = plane(1)
fig = Cylinder([-670,8,5],[-730,30,50],1.6)
fig = Cylinder([-600,390,5],[-645,450,50],1.6)
fig = Cylinder([-542.3,-360.3,148.6],[-532,-395,1],1.6)
fig,x_,y_,z_ = robot_movement(thetas,thetae,10)
print(x_,y_,z_)
for i in range(len(x_)-1):
    X1,Y1,Z1,X2,Y2,Z2 = cylinder_line([x_[i],y_[i],z_[i]],[x_[i+1],y_[i+1],z_[i+1]],10)
    for j in range(len(X1)):
        d = go.Scatter3d(x=[X1[j],X2[j]],y=[Y1[j],Y2[j]],z=[Z1[j],Z2[j]])
        fig.add_trace(d)
# for i in range(len(x_p)-1):
#     pt1 = [x_p[i],y_p[i],z_p[i]]
#     pt2 = [x_p[i+1],y_p[i+1],z_p[i+1]]
#     fig = collision_line(pt1,pt2)
#     v = np.subtract(pt2,pt1)
#     mag = norm(v)
#     unit_v = [v[0]/mag,v[1]/mag,v[2]/mag]
#     unit_v = asarray(unit_v)
#     p = pt1 + (mag*v)
#     p_ = -(pt1 + (mag*v))
#     # x_move1.append(p[0])
#     # y_move1.append(p[1])
#     # z_move1.append(p[2])
            
#     data = go.Scatter3d(x=[pt1[0],p[0]], y=[pt1[1],p[1]], z=[pt1[2],p[2]],marker = dict(opacity = 0.99))
#     fig.add_trace(data)
#     data = go.Scatter3d(x=[pt1[0],p_[0]], y=[pt1[1],p_[1]], z=[pt1[2],p_[2]],marker = dict(opacity = 0.99))
#     fig.add_trace(data)
fig.show()

# x = raw_input("Enter if you want plane or not:")
# if (x == "Y" or x == "y"):
#     h = float(input())
#     fig = plane(h)
#     fig.show()
# else:
#     pass

# while True:
#     x = raw_input("Enter if you want to enter a cylinder or not:")
#     if (x == "Y" or x == "y"):
#         pt1 = list(map(float,raw_input().split()))
#         pt2 = list(map(float,raw_input().split()))
#         r = float(input())
#         fig = Cylinder(pt1,pt2,r)
#         fig.show()
#         continue
#     else:
#         pass

#     # z = raw_input("Enter if you want to enter a collision line or not:")
#     # if (z == "Y" or z == "y"):
#     #     p1 = list(map(float,raw_input().split()))
#     #     p2 = list(map(float,raw_input().split()))
#     #     fig = collision_line(p1,p2)
#     #     fig.show()
#     #     continue
#     # else:
#     #     pass

#     z = raw_input("Enter if you want robot movement or not:")
#     if (z == "Y" or z == "y"):
#         thetas = list(map(float,raw_input().split()))
#         thetae = list(map(float,raw_input().split()))
#         frames_ = int(input())
#         fig = robot_movement(thetas,thetae,frames_)
#         fig.show()
#         continue
#     else:
#         break


