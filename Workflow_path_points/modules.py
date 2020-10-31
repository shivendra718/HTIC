#import Packages:
import urx
import math
import math3d as m3d
import numpy as np
import time
import quaternion
np.set_printoptions(suppress=True)
from scipy.spatial.transform import Rotation as R
from decimal import *
import plotly.graph_objects as go

from CollisionFunction import * 


#INVERSE KINEMATICS:

def DHTable2HomTrans(DHTable):
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i in range(np.shape(DHTable)[0]):
        T1 = np.dot(T,DH2HomTrans(DHTable[i]))
        T = T1
    return T

def DH2HomTrans(DHparams):
    [al,a,d,th] = DHparams
    T = [[np.cos(th),-np.sin(th)*np.cos(al),np.sin(th)*np.sin(al),a*np.cos(th)],[np.sin(th),np.cos(th)*np.cos(al),-np.cos(th)*np.sin(al),a*np.sin(th)],[0,np.sin(al),np.cos(al),d],[0,0,0,1]]
    return T

def sol2Rik(x,y,l1,l2):
    a = -2.0*l1*x
    b = -2.0*l1*y
    c = l1**2.0 - l2**2.0 + x**2.0 + y**2.0
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta2 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta12 = np.arctan2((y - l1*np.sin(i))/l2,(x - l1*np.cos(i))/l2)
        theta2[j] = theta12 - i
        j = j+1
    t1t2 = np.column_stack((theta1, theta2))
    return t1t2

# IK Solver:
def solUR5ik(p,R):
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997 #+ 0.025
    d6 = 0.0996 #+ 0.08165
    [x,y,z] = p
    [[nx,ox,ax],[ny,oy,ay],[nz,oz,az]] = R
    a = ay*d6 - y
    b = -ax*d6 + x
    c = -d4
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta5 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta5[j] = [np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6),-np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6)]
        j = j+1
    
    
    t1t5 = [[theta1[0],theta5[0][0]],[theta1[0],theta5[0][1]],[theta1[1],theta5[1][0]],[theta1[1],theta5[1][1]]]
    theta6 = [None]*np.shape(t1t5)[0]
    j = 0
    for i in range(np.shape(t1t5)[0]):
        theta6[j] = [np.arctan2((oy*np.cos(t1t5[i][0])-ox*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]),(-ny*np.cos(t1t5[i][0])+nx*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]))]
        j = j+1     
    t1t5t6 = np.hstack((t1t5,theta6))
    k = 0
    t2t3t4 = [None]*np.shape(t1t5)[0]*2  
    TUR5 = np.vstack((np.hstack((R,np.array(p).reshape(3,1))),[0,0,0,1]))
    for i in range(np.shape(t1t5t6)[0]):
        T01 = DHTable2HomTrans([[np.pi/2,0,d1,t1t5t6[i][0]]])
        T56 = DHTable2HomTrans([[-np.pi/2,0,d5,t1t5t6[i][1]],[0,0,d6,t1t5t6[i][2]]])
        T3R = np.dot(np.dot(np.linalg.inv(T01),TUR5),np.linalg.inv(T56))
        
        theta234 = np.arctan2(T3R[1][0],T3R[0][0])
        theta23 = sol2Rik(T3R[0][3],T3R[1][3],a2,a3)
        for j in range(np.shape(theta23)[0]):
            theta4 = theta234 - theta23[j][0] - theta23[j][1]
            t2t3t4[k]=[theta23[j][0],theta23[j][1],theta4]
            k = k+1
    t1 = np.array([val for val in t1t5 for _ in (0, 1)])[:,0]
    t5 = np.array([val for val in t1t5 for _ in (0, 1)])[:,1]
    t6 = np.array([val for val in theta6 for _ in (0, 1)])
    ikUR5 = np.hstack((t1.reshape(8,1),t2t3t4,t5.reshape(8,1),t6.reshape(8,1)))
    return ikUR5

#********************SUBORDINATES*******************************
def Angle1D2FromEntTar_Refined(Entry,Target,radius):
    theta1 = math.acos( (Entry[2] - Target[2]) / radius )
    checker_1 = (Entry[1] - Target[1]) / (radius * math.sin(theta1))
    if checker_1 > 1:
        checker_1 = 1
    elif checker_1 < -1:
        checker_1 = -1
    theta2 = math.asin(checker_1)
    try :
        checker_2 = (Entry[0] - Target[0]) / (radius * math.sin(theta1))
        if checker_2 > 1:
            checker_2 = 1
        elif checker_2 < -1:
            checker_2 = -1
        theta21 = math.acos(checker_2)
    except ValueError:
        print('PosePtim...choose 2pi or pi')
        value = raw_input()
        theta21 = float(value)
    return math.degrees(theta1),math.degrees(theta2),math.degrees(theta21)

# Computing orientation for the Entry Target Values:
def OrientMatix(entry,target):
    
    v_z = np.asarray(target) - np.asarray(entry)
    #unit vector
    denom = np.sqrt( np.square(v_z[0]) + np.square(v_z[1]) + np.square(v_z[2]))
    vector_z = -( v_z / denom)       
    
    #vector y:
    element1 = -( ((vector_z[0]*entry[0]) + (vector_z[1]*entry[1]))  / vector_z[2])
    v_x = np.asarray( [entry[0],entry[1],element1] )
    denom1 = np.sqrt( np.square(v_x[0]) + np.square(v_x[1]) + np.square(v_x[2]))
    vector_x = v_x / denom1   
    
    #vector z:
    vector_y = np.asarray ( np.cross(vector_z,vector_x))
    
    return vector_x,vector_y,vector_z

def OrientMatixRotation(entry,target,theta):
    
    #vector y:
    v_z = np.asarray(target) - np.asarray(entry)
    #unit vector
    denom = np.sqrt( np.square(v_z[0]) + np.square(v_z[1]) + np.square(v_z[2]))
    vector_z = -( v_z / denom)
        
    #vector z:
    element1 = -( ((vector_z[0]*entry[0]) + (vector_z[1]*entry[1]))  / vector_z[2])
    v_x = np.asarray( [entry[0],entry[1],element1] )
    denom1 = np.sqrt( np.square(v_x[0]) + np.square(v_x[1]) + np.square(v_x[2]))
    vector_x = v_x / denom1
    
    #Rotation shift:
    #Rodrigues form
    comp1 = vector_x * np.cos(theta)
    comp2 = np.cross(vector_z,vector_x) * np.sin(theta)
    comp3 = (vector_z * (np.dot(vector_z,vector_x)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    vector_x = Rodgriues
    
    #vector x:
    vector_y = np.asarray ( np.cross(vector_z,vector_x))
    

    return vector_x,vector_y,vector_z

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


def PoseOptimthe456(vec1,vec2):
    vec = np.asarray(vec2) - np.asarray(vec1)
    mag = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
    unit = (vec[0]/mag,vec[1]/mag,vec[2]/mag)
    #Define Basis
    x=np.array((1,0,0))
    y=np.array((0,1,0))
    z=np.array((0,0,1))
    #Angle
    xangle=np.arccos(np.dot(unit,x)/(np.linalg.norm(unit)*np.linalg.norm(x)))
    yangle=np.arccos(np.dot(unit,y)/(np.linalg.norm(unit)*np.linalg.norm(y)))
    zangle=np.arccos(np.dot(unit,z)/(np.linalg.norm(unit)*np.linalg.norm(z)))
#     print("X : {} | Y : {} | Z : {}".format(((xangle*180)/math.pi),((yangle*180)/math.pi),((zangle*180)/math.pi)))
    finangles = [xangle,yangle,zangle]
    return finangles

def joints_2_degree(cjv):
    deg = []
    for i in range(len(cjv)):
        deg.append((cjv[i]*180)/math.pi)
    return deg

def degree2radians(thetas):
    rad = []
    for i in range(len(thetas)):
        rad.append((thetas[i] * math.pi)/180)
    return rad

def conversion(joint_angle):
    dumy = [0,-2*math.pi,2*math.pi,0,0,0]
    ss = np.add(joint_angle,dumy)
    return ss

def thetacheck(pose):
    status = 'False'
    if int(pose[0]) in range(-90,100):
        if int(pose[1]) in range(-120,-10):
            if int(pose[2]) in range(0,150):
                pos = JointLocations(degree2radians(pose))
                angl1 = PoseOptimthe456(pos[3],pos[4])
                angl2 = PoseOptimthe456(pos[4],pos[5])
                
                angl1R = PoseOptimthe456(pos[3],pos[2])
                angl2R = PoseOptimthe456(pos[5],pos[4])
                if angl1[2] >= 2.25 and angl1[2] <= math.pi: 
                    if angl2[1] >= 1.5 and angl2[1] <= math.pi:
                        if angl2[2] >= 1.25 and angl2[2] <= math.pi:
                            if np.abs(angl1R[1] - angl2R[1]) >= 0:
                                status = 'True' 

    return status
    
def anglesarrayModified(set_s):
    flag = 0
    join_values = []
    var = joints_2_degree(set_s)
    for j in var:
        if str(j) == 'nan':
            flag = 1
            break 
    if flag == 0:
        checker = thetacheck(var)
        if checker == 'True':
            join_values = set_s
        else:
            ret_joint = conversion(set_s)
            checker2 = thetacheck(joints_2_degree(ret_joint))
            if checker2 == 'True':
                join_values = ret_joint
            else:
                join_values = ret_joint
#                 "Exceeding joint limits"
                return [join_values,1]
        return [join_values,0]
    else:
#         "Found Nan"
        return [join_values,2]


#Get Initial and Final Orientation:

def GetInitialFinalOrientation(Entry,Target):

    #Variables:
    IK_Counter = 0
    looper = 0
    lop_inc = 0
    count = 1
    mark_status = 0
    flagger = 0

    #Offset:
    pers_scale = np.array([0,0,0,1])
    offset_xyz = np.array([0,0,0])
    Rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    # Rotation = np.array([[-0.9995041, -0.0312873,  0.0035582],[-0.0034909, -0.0022056, -0.9999915],[0.0312949, -0.9995080,  0.0020953]])
    a = np.hstack((Rotation,offset_xyz.reshape(3,1)))
    Trans_6to7 = np.vstack((a,pers_scale))
    TT = np.linalg.inv(Trans_6to7)

    EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 
    Entry_Modified = EM 
    Target_Modified = TM   
    
    x,y,z = OrientMatix(Entry_Modified,Target_Modified)
    
    # while(looper == 0):
        # x,y,z = OrientMatix(Entry_Modified,Target_Modified)
    # Orientation = [y,z,x]
    Orientation = [x,y,z]
    Orientation_Transpose = np.transpose(Orientation)

    #Input matrix
    pos = np.array(Entry_Modified)
    ore = np.array(Orientation_Transpose)
    H_half = np.hstack((ore,pos.reshape((3,1))))
    Input_H  = np.vstack((H_half,pers_scale))

    #Transformation :  6-7
    # T0_6 = np.dot(Input_H,TT)
    T0_6 = Input_H
    ORIENTATION = T0_6[0:3,0:3]
    POSITION = T0_6[0:3,3]


    InitialOrientation = np.array([[0.2802784,  0.7650343, -0.5797988],[-0.5102854, -0.3928504, -0.7650343],[ -0.8130516,  0.5102854,  0.2802783 ]])
    return InitialOrientation,ORIENTATION


#***********Time step Orientation*************************

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return angle

def rodrot(vec,axis,theta):
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def TimeStepAngles(vec1,vec2,theta,Length):
    Stack = []
    vec3 = np.cross(vec1,vec2)
    vec3 = Vec2UnitVec(vec3)
    Inc = float(Decimal(theta)/Decimal(Length/2))
    AngleChange = list(np.arange(0,theta,Inc))
    RepeatAngle = list(np.repeat(theta,(Length - len(AngleChange))))
    Angles = AngleChange + RepeatAngle
    
    for i in range(Length):
        Vec = rodrot(vec1,vec3,Angles[i])
        Stack.append(Vec)
    return Stack

def RetriveTimeStepOrientation(IO,EO,Length):
    TSOrient = []
    x,y,z = IO[:,0],IO[:,1],IO[:,2]
    xe,ye,ze = EO[:,0],EO[:,1],EO[:,2]
    theta1 = AnglebtwnVecs(x,xe)
    theta2 = AnglebtwnVecs(y,ye)
    theta3 = AnglebtwnVecs(z,ze)
    
    C_x = TimeStepAngles(x,xe,theta1,Length)
    C_y = TimeStepAngles(y,ye,theta2,Length)
    C_z = TimeStepAngles(z,ze,theta3,Length)
    
    for i in range(len(C_x)):
        TSOrient.append( np.transpose(np.array(([C_x[i],C_y[i],C_z[i]]))))
        
    return TSOrient

#***********Time step Orientation*************************


#Main thread : Retrive IK Solutions:

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

def all_main_run(Entry,Target,theta,solution):
    
    IK_Counter = 0
    looper = 0
    lop_inc = 0
    count = 1
    mark_status = 0
    flagger = 0

    #Adding offset in Rotation and Translation
    pers_scale = np.array([0,0,0,1])

    offset_xyz = np.array([0,0,0]) #Translation shift
    Rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    # # Rotation = np.array([[-0.9995041, -0.0312873,  0.0035582],[-0.0034909, -0.0022056, -0.9999915],[0.0312949, -0.9995080,  0.0020953]])
    
    a = np.hstack((Rotation,offset_xyz.reshape(3,1)))
    Trans_6to7 = np.vstack((a,pers_scale))
    TT = np.linalg.inv(Trans_6to7)

    EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 

    # # Origin = np.array([-0.6046110713707592,-0.13289210398831858,0.45235422635752986])

    Entry_Modified = EM 
    Target_Modified = TM 

    
    x,y,z = OrientMatixRotation(Entry_Modified,Target_Modified,theta)
    # # x,y,z = OrientMatix(Entry_Modified,Target_Modified)
    Orientation = [y,z,x]
    # Orientation = [x,y,z]   
    Orientation_Transpose = np.transpose(Orientation)

    #Input matrix
    pos = np.array(Entry_Modified)
    ore = np.array(Orientation_Transpose)
    H_half = np.hstack((ore,pos.reshape((3,1))))
    Input_H  = np.vstack((H_half,pers_scale))

    #Transformation :  6-7
    T0_6 = np.dot(Input_H,TT)
    # T0_6 = Input_H
    OREINTATION = T0_6[0:3,0:3]
    POSITION = T0_6[0:3,3]

    #Computing Anlges:

    angles = solUR5ik(POSITION,OREINTATION)
        
    
    jv = angles[solution]
    # dumy = [0,-2*math.pi,2*math.pi,0,0,0]
    # jv = np.add(jv,dumy)
    
    flag_ = 0   
    print(jv)


    if jv[5] < 0:
        jv[5] = (2*math.pi)  + jv[5]

        # print(joints_2_degree(jv))

    return jv
    


def RetriveIKSolutions(Path,Orientation,Flag,angleValue,solution):
    
    x_move = []
    y_move = []
    z_move = []
    
    for i in range(len(Path[0])):

        PathPoint = [Path[0][i],Path[1][i],Path[2][i]]
        CurrentOreint = Orientation[i]
        Entry = np.asarray(PathPoint)
        z_axis = np.asarray(CurrentOreint[:,2])
        tp = ((-z_axis * 10) + Entry) / 1000
        Target = np.asarray(tp) * 1000

        if Flag == 0:
            joint_values = all_main_run(Entry,Target,0,solution)
        else :
            joint_values = all_main_run(Entry,Target,angleValue,solution)
        
        #FK part
        Fk_values = JointLocations(joint_values)
        x_,y_,z_ = SepPoints(np.asarray(Fk_values))

        x_move.append(x_)
        y_move.append(y_)
        z_move.append(z_)
    
    return x_move,y_move,z_move








            
