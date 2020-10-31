import numpy as np
import plotly.graph_objects as go
from numpy import *
from scipy.linalg import norm
#import cylinder as c

##################### for plane
def plane(height):
    
    x= 80+np.linspace(-1, 100, 75)
    y= np.linspace(0, 100, 100)
    z= height*np.ones((100,75))
    return x,y,z

x1,y1,z1 = plane(1)
mycolorscale = [[0, '#aa9ce2'],
                [1, '#aa9ce2']]
surf = go.Surface(x=x1, y=y1, z=z1, colorscale=mycolorscale, showscale=False)
layout = go.Layout(width=1800,
                  scene_camera_eye_z=0.75,
                  title="Start Title",
                updatemenus=[dict(
                type="buttons",
                buttons=[dict(label="Play",
                          method="animate",
                          args=[None])])])
fig = go.Figure(data=[surf], layout=layout)



############################# for cuboids   

data=go.Mesh3d(
        # 8 vertices of a cube
        x=[50, 50, 70, 70, 50, 50, 70, 70],
        y=[25, 45, 45, 25, 25, 45, 45, 25],
        z=[0, 0, 0, 0, 4, 4, 4, 4],
        colorbar_title='z',
       
         #Intensity of each vertex, which will be interpolated and color-coded
        #intensity = np.linspace(0, 1, 8, endpoint=True),
        #i, j and k give the vertices of triangles
        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        name='y',
        showscale=True
    )

fig.add_trace(data)


##################### for cylinder of the robot

def cylinder(r, h, a =0, nt=100, nv =50):
    """
    parametrize the cylinder of radius r, height h, base point a
    """
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = 70+r*np.cos(theta)
    y= 35+r*np.sin(theta)
    z = 4+v
    return x, y, z

def boundary_circle(r, h, nt=100):
    """
    r - boundary circle radius
    h - height above xOy-plane where the circle is included
    returns the circle parameterization
    """
    theta = np.linspace(0, 2*np.pi, nt)
    x= 70+r*np.cos(theta)
    y= 35+r*np.sin(theta)
    z = 4+h*np.ones(theta.shape)
    return x, y, z
r1 = 1.5
a1 = 0
h1 = 2


x1, y1, z1 = cylinder(r1, h1, a=a1)

colorscale = [[0, 'red'],
                [1, 'red']]

cyl1 = go.Surface(x=x1, y=y1, z=z1,
                 colorscale=colorscale,
                 showscale=False,)
                 
xb_low, yb_low, zb_low = boundary_circle(r1, h=a1)
xb_up, yb_up, zb_up = boundary_circle(r1, h=a1+h1)

bcircles1 =go.Scatter3d(x = xb_low.tolist()+[None]+xb_up.tolist(),
                        y = yb_low.tolist()+[None]+yb_up.tolist(),
                        z = zb_low.tolist()+[None]+zb_up.tolist(),
                        mode ='lines',
                        line = dict(color='red', 
                                    width=2),
                        
                        showlegend=False)


#fig.add_trace(go.Scatter(x=[50,50],y=[0,2], name="linear",line_shape='linear'))


fig.add_trace(cyl1)
fig.add_trace(bcircles1)


############################################################################ for robot coordinates

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


thetas=[0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
thetae=[0.3803219795227051,-0.6225808423808594,1.3531478087054651,-0.6644571584514161,0.43364858627319336,3.082611560821533]  #theta1 rotation compensation
theta=[]


for i in range(0,len(thetas)):
    b = thetae[i]-thetas[i]
    theta.append(b)


temp_joint_locations=thetas

sl = JointLocations(temp_joint_locations)


frames_ = 10

joint_coordinates = []
coordinates = []
inc_=np.divide(theta,frames_)

x_move=[]
y_move=[]
z_move=[]

v0 = [0,0,0.1625]
v1 = [0,-0.1333,0.1625]
V = np.subtract(v1,v0)

v3 = [-2.602374e-17,0,0.5875]
axis = np.subtract(v3,v0)
axis_mag = np.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
axis_unit = (axis[0]/axis_mag,axis[1]/axis_mag,axis[2]/axis_mag)
axis_unit = np.asarray(axis_unit)
h=[0.,0.,0.]
h=np.asarray(h)
x_points = []
y_points = []
z_points = []           


for j in range(frames_):
    temp_joint_locations = np.add(temp_joint_locations,inc_)
    joint_coordinates.append(np.array(temp_joint_locations))
    

for i in range(0,frames_):
    h += np.asarray(V*np.cos(inc_[0])+np.cross(axis_unit,V)*np.sin(inc_[0])+ axis_unit*(np.dot(axis_unit,V))*(1-np.cos(inc_[0])))
    x_points.append(h[0])
    y_points.append(h[1])
    z_points.append(h[2])
    
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
    
    x_=np.array(x_)*-70
    y_=np.array(y_)*-35
    z_=np.array(z_)*4
    x_=list(x_)
    y_=list(y_)
    z_=list(z_)
    x_=list(np.asarray(x_)+70)
    y_=list(np.asarray(y_)+35)
    z_=list(np.asarray(z_)+5.35)


    x_move.append(x_)
    y_move.append(y_)
    z_move.append(z_)


x_p,y_p,z_p = SepPoints(sl)

x_p.insert(1,0)
x_p.insert(2,-2.602374e-17)

y_p.insert(1,-0.1333)
y_p.insert(2,-0.1333)

z_p.insert(1,0.1625)
z_p.insert(2,0.5875)

x_p=np.array(x_p)*-70
y_p=np.array(y_p)*-35
z_p=np.array(z_p)*4
x_p=list(x_p)
y_p=list(y_p)
z_p=list(z_p)
x_p=list(np.asarray(x_p)+70)
y_p=list(np.asarray(y_p)+35)
z_p=list(np.asarray(z_p)+5.35)


###################################### for robot frames


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
fig.add_trace(data)


fig.update_layout(scene=dict(zaxis=dict(range=[-3,9],autorange=False),       ###########[-3,9]
                       yaxis=dict(range=[0, 80],autorange=False),
                   xaxis=dict(range=[0,130],autorange=False)))               #######[40,130]
fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))


for i in range(frames_):
    x2,y2,z2 = plane(1)
    mycolorscale = [[0, '#aa9ce2'],
                [1, '#aa9ce2']]

    surf = go.Surface(x=x2, y=y2, z=z2, colorscale=mycolorscale, showscale=False)
    fig.add_trace(surf)
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

  
    
    fig.frames=tuple(b)
fig.add_trace(data)


####################### for sphere and cylinder


start_end = []
radius = []
step_ = []

def Cylinder(pt1,pt2,r,step):
    data = go.Scatter3d(x=[pt1[0],pt2[0]], y=[pt1[1],pt2[1]], z=[pt1[2],pt2[2]],
                        line = dict(width=50))
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
        y = newpts[1]+r*outer(sin(theta),sin(phi))      ############## x,y,z are parametric form of spheres.
        z = newpts[2]+r*outer(ones(100),cos(phi))

        data=go.Surface(
            x=x,
            y=y,
            z=z,
            opacity=0.3
        )





        
        

    
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

    x,y,z = [v1[i] + unit_v[i] * v + r * np.sin(theta1) * n1[i] + r * np.cos(theta1) * n2[i] for i in [0, 1, 2]]   ############ parametric form of the cylinder
    
    
    cyl1 = go.Surface(x=x, y=y, z=z,
                #colorscale = colorscale,
                 showscale=False,
                 opacity=0.5)
    #fig.add_trace(cyl1)

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
                p1[2]*(1-x_2) + x_2*p2[2]]               ################## sol1 and sol2 are solutions of the given quadratic equations ie, these are the intersection points

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

Cylinder([95,50,1],[98,60,3],1.6,7)
Cylinder([100,50,1],[110,60,2],1.6,7)
collision_line([3,7,3],[7,8,7])
fig.show()