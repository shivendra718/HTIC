#import Packages:
import urx
import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *
import plotly.graph_objects as go


#Functions

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


def conversion(joint_angle):
    dumy = [0,-2*math.pi,2*math.pi,0,0,0]
    ss = np.add(joint_angle,dumy)
    return ss

def OrientMatix(entry,target):
    v_y = np.asarray(target) - np.asarray(entry)
    #unit vector
    denom = np.sqrt( np.square(v_y[0]) + np.square(v_y[1]) + np.square(v_y[2]))
    vector_y = -( v_y / denom)       
    #vector y:
    element1 = -( ((vector_y[0]*entry[0]) + (vector_y[1]*entry[1]))  / vector_y[2])
    v_z = np.asarray( [entry[0],entry[1],element1] )
    denom1 = np.sqrt( np.square(v_z[0]) + np.square(v_z[1]) + np.square(v_z[2]))
    vector_z = v_z / denom1   
    #vector z:
    vector_x = np.asarray ( np.cross(vector_y,vector_z))
    return vector_x,vector_y,vector_z

def PositionRetrival(pos_,orien):
    #Tranformation : T 0-7
    PP1 = [float(Decimal(pos_[0]) / Decimal(1000)),float(Decimal(pos_[1]) / Decimal(1000)),float(Decimal(pos_[2]) / Decimal(1000))] 
    pers_scale = np.array([0,0,0,1])
    pos = np.array(PP1)
    ore = np.array(orien)
    H_half = np.hstack((ore,pos.reshape((3,1))))
    Input_H  = np.vstack((H_half,pers_scale))

    #Adding offset in Rotation and Translation : T 6-7

    offset_xyz = np.array([-0.00012,0.05213, 0.08876]) #Translation shift
    Rotation = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
    

    a = np.hstack((Rotation,offset_xyz.reshape(3,1)))
    Trans_6to7 = np.vstack((a,pers_scale))
    TT = np.linalg.inv(Trans_6to7)

    #Transformation :  6-7
    T0_6 = np.dot(Input_H,TT)
    OREINTATION = T0_6[0:3,0:3]
    Pos = T0_6[0:3,3]
    return Pos,OREINTATION

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


def PathContinuityVerification(JJ):

    JV = []
    Prev_value = [0,-math.pi/2,math.pi/2,0,math.pi/2,math.pi]
    for i in range(len(JJ)):

        current_value = JJ[i]
        joint1 = Prev_value[1] - current_value[1]
        joint6 = Prev_value[5] - current_value[5]
        
        if (np.abs(joint1) > 1.5 ):
            current_value = conversion(current_value)
        if (np.abs(joint6) > 1.5):
            current_value[5] = (2*math.pi)  + current_value[5]
            
        print(str(list(current_value)) + ",")
        Prev_value = current_value
        JV.append(current_value)

    return JV
