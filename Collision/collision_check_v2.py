# This code is to find out of a line intersects a cylinder in any point. Spheres are approximated as cylinder.

import plotly.graph_objects as go
import numpy as np
from numpy import *
from scipy.linalg import norm

fig = go.Figure()

start_end = []
radius = []
step_ = []

def Cylinder(pt1,pt2,r):
    step = 7
    data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]])
    
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
        fig.add_trace(data)

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
    fig.add_trace(cyl1)

    start_end.append([v1,v2])
    radius.append(r)
    step_.append(step)
    return fig
    
    

def collision_line(p1,p2):
    data = go.Scatter3d(x=[p1[0],p2[0]], y=[p1[1],p2[1]], z=[p1[2],p2[2]],line = dict(width=10))
    fig.add_trace(data)
    # newpts_opp = []
    p1_ = p1
    p2_ = p2
    v_line = np.subtract(p2,p1)
    mag1 = norm(v_line)
    newpts_ =  p1 + (mag1*v_line)
    newpts_opp = (p1 - (mag1*v_line))
    # data = go.Scatter3d(x=[newpts_opp[0],newpts_[0]], y=[newpts_opp[1],newpts_[1]], z=[newpts_opp[2],newpts_[2]])
    # fig.add_trace(data)
    # data = go.Scatter3d(x=[p1[0],newpts_opp[0]], y=[p1[1],newpts_opp[1]], z=[p1[2],newpts_opp[2]])
    # fig.add_trace(data)


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
            #print(dist_v)
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

        
            discriminant2 = (b2**2 - 4 * a2 * c2)
            # print(discriminant2)


        
            discriminant1 = b1**2 - 4 * a1 * c1
            
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

            # print(sol11,sol22)
           

            
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
                dist1 = sqrt((sol11[0]-p1_[0])**2 + (sol11[1]-p1_[1])**2 + (sol11[2]-p1_[2])**2)
                dist2 = sqrt((sol11[0]-p2_[0])**2 + (sol11[1]-p2_[1])**2 + (sol11[2]-p2_[2])**2)
                # print(dist1,dist2)
                if dist1<dist2:
                    dist5_ = dist1
                    dist_array_lni.append(dist5_)
                else: 
                    dist6_ = dist2
                    dist_array_lni.append(dist6_)
            elif ((x_22 > 1 or x_22 < 0) and not(x_12 > 12 or x_12 < 0)):
                dist1 = sqrt((sol22[0]-p1_[0])**2 + (sol22[1]-p1_[1])**2 + (sol22[2]-p1_[2])**2)
                dist2 = sqrt((sol22[0]-p2_[0])**2 + (sol22[1]-p2_[1])**2 + (sol22[2]-p2_[2])**2)
                # print(dist1,dist2)
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
#     z = raw_input("Enter if you want to enter a collision line or not:")
#     if (z == "Y" or z == "y"):
#         p1 = list(map(float,raw_input().split()))
#         p2 = list(map(float,raw_input().split()))
#         fig = collision_line(p1,p2)
#         fig.show()
#         continue
#     else:
#         break

fig = Cylinder([2,2,1],[6,8,6],1.6)
# fig = Cylinder([2,1,2],[9,8,9],1.6)

fig = collision_line([5,5,1],[9,9,1])
fig.show()


    

