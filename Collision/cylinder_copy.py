import plotly.graph_objects as go
import numpy as np
from numpy import *
from scipy.linalg import norm

fig = go.Figure()

start_end = []
radius = []
step_ = []

def Cylinder(pt1,pt2,r,step):
    data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]])
    fig.add_trace(data)
    v = subtract(pt2,pt1)
    mag = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
    unit_v=asarray(unit_v)
##################### sphere

    dist = sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2 + (pt2[2]-pt1[2])**2)
    v1 = pt1
    v2 = pt2
    pdist = dist/step
    #print(pdist)
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





        fig.add_trace(data)
        

    
################ cylinder

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
                #colorscale = colorscale,
                 showscale=False,
                 opacity=0.5)
    fig.add_trace(cyl1)
    return fig

    start_end.append([v1,v2])
    radius.append(r)
    step_.append(step)
    

def collision_line(p1,p2):
    data = go.Scatter3d(x=[p1[0],p2[0]], y=[p1[1],p2[1]], z=[p1[2],p2[2]])
    fig.add_trace(data)

    for i in range(len(start_end)):
        newpts_array = []
        v = np.subtract(start_end[i][1],start_end[i][0])
        mag = norm(v)
        unit_v = [v[0]/mag, v[1]/mag, v[2]/mag]
        unit_v=asarray(unit_v)
        pt1 = start_end[i][0]
        pt2 = start_end[i][1]
        r = radius[i]
        step = step_[i]
        print(i+1,"Cylinder")
        pdist = mag/step
        dist_array = []
        for i in range(0,step):
            dist_v = pdist*unit_v
            #print(dist_v)
            newpts = pt1 + dist_v
            newpts_array.append(newpts)
            pt1 = newpts
            newpts=list(newpts)
            


            a = (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2
            b = 2*((p2[0]-p1[0])*(p1[0]-newpts_array[i][0]) + (p2[1]-p1[1])*(p1[1]-newpts_array[i][1]) + (p2[2]-p1[2])*(p1[2]-newpts_array[i][2]))
            c = (newpts_array[i][0]**2 + newpts_array[i][1]**2 + newpts_array[i][2]**2 + p1[0]**2 + p1[1]**2 + p1[2]**2 - 2*(newpts_array[i][0]*p1[0] + 
                                                                        newpts_array[i][1]*p1[1] + newpts_array[i][2]*p1[2]) - r**2)


            
            discriminant = b**2 - 4 * a * c
            if discriminant >= 0:
                x_1=(-b+math.sqrt(discriminant))/(2*a)
                x_2=(-b-math.sqrt(discriminant))/(2*a)
            else:
                x_1= complex((-b/(2*a)),math.sqrt(-discriminant)/(2*a))
                x_2= complex((-b/(2*a)),-math.sqrt(-discriminant)/(2*a))

        #    

            sol1 = [p1[0]*(1-x_2) + x_2*p2[0],
                p1[1]*(1-x_2) + x_2*p2[1],
                p1[2]*(1-x_2) + x_2*p2[2]]

            sol2 = [p1[0]*(1-x_1) + x_1*p2[0],
                p1[1]*(1-x_1) + x_1*p2[1],
                p1[2]*(1-x_1) + x_1*p2[2]]

            
            if (discriminant < 0):
                q=0
                
            elif((x_2>1 or x_2<0) and (x_1>1 or x_1<0)):
                o=0
            elif (not(x_2 > 1 or x_2 < 0) and (x_1 > 1 or x_1 < 0)):
                dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
                dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
                if dist1<dist2:
                    dist1_ = dist1
                    dist_array.append([dist1_,sol1])
                else: 
                    dist2_ = dist2
                    dist_array.append([dist2_,sol1])
            elif ((x_2 > 1 or x_2 < 0) and not(x_1 > 1 or x_1 < 0)):
                dist1 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
                dist2 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
                if dist1<dist2:
                    dist3_ = dist1
                    dist_array.append([dist3_,sol2])
                else: 
                    dist4_ = dist2
                    dist_array.append([dist4_,sol2])
            else:
                dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
                dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
                dist3 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
                dist4 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
                if (dist1<dist2 and dist1<dist3 and dist1<dist4):
                    dist5_ = dist1
                    dist_array.append([dist5_,sol1])
                elif (dist2<dist1 and dist2<dist3 and dist2<dist4):
                    dist6_ = dist2
                    dist_array.append([dist6_,sol1])
                elif (dist3<dist1 and dist3<dist2 and dist3<dist4):
                    dist7_ = dist3
                    dist_array.append([dist7_,sol2])
                elif (dist4<dist1 and dist4<dist2 and dist4<dist3):
                    dist8_ = dist4
                    dist_array.append([dist8_,sol2])
        dist_array.sort()
        if (dist_array != []):
            print("Intersection point:",dist_array[0][1])
            print("Shortest Distance:",dist_array[0][0])
        else:
            print("No intersection point")
    return fig


# Cylinder([2,1,1],[5,6,5],1.6,7)
# Cylinder([2,2,1],[2,2,8],1.6,8)
# collision_line([5,5,5],[7,7,7])

# fig.show()
            