import numpy as np
from numpy import *
import math
from decimal import *
from modules import *

from LineToCylinder import *

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def FindIntersectionPoints(x1,y1,z1,x2,y2,z2,T):
    PointX = x1 + T* (x2 - x1)
    PointY = y1 + T* (y2 - y1)
    PointZ = z1 + T* (z2 - z1)
    return [PointX,PointY,PointZ] 

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
#     print("Angle : {}".format((angle*180)/math.pi))
    return angle

def EuclideanDistance(vec1,vec2):
    E2Distance = np.sqrt( np.square(vec1[0]- vec2[0]) + np.square(vec1[1]- vec2[1]) + np.square(vec1[2]- vec2[2]))
    return E2Distance

def ComputeTransformation(xpoints,ypoints,zpoints,TMat,flag):
    Xp = []
    Yp = []
    Zp = []

    for i in range(len(xpoints)):
        vect = [xpoints[i],ypoints[i],zpoints[i]]
        if flag == 1:
            TransformedCodinates1 = np.dot(TMat[0],vect)
            TransformedCodinates2 = np.dot(TMat[1],TransformedCodinates1)
        else : 
            TransformedCodinates2 = np.dot(TMat,vect)
            
        Xp.append(TransformedCodinates2[0])
        Yp.append(TransformedCodinates2[1])
        Zp.append(TransformedCodinates2[2])
    
    xp = np.reshape(Xp,(100,100))
    yp = np.reshape(Yp,(100,100))
    zp = np.reshape(Zp,(100,100))
    return xp,yp,zp

def CheckInisideorOutsideEllipsoid(p1,mp,Trans):

    vec = np.asarray(p1) - np.asarray(mp)
    result = np.dot(np.dot(vec,Trans),vec)
    return result

def CheckPointInsideEllipsoid(point,mat):
    result = np.dot(np.dot(point,mat),point)
    return result


def EllipsoidDetails(Entry,Target):
    
    ModifiedVector = [Entry[0],Entry[1],Target[2]]
    MidPoint = [float(Decimal(Entry[0] + Target[0])/Decimal(2)),float(Decimal(Entry[1] + Target[1])/Decimal(2)),float(Decimal(Entry[2] + Target[2])/Decimal(2))]
   
    MajorAxisRadius = EuclideanDistance(Entry,MidPoint)
    MinorAxisRadius = 20.12
   
    DVector1 = np.asarray(Entry) - np.asarray(Target)
    DVector2 = np.asarray(ModifiedVector) - np.asarray(Target)
    
    Basis_X = [1,0,0]
    Basis_Z = [0,0,1]
    
    the1 = AnglebtwnVecs(DVector1,Basis_Z)
    the2 = AnglebtwnVecs(DVector2,Basis_X)
    
    theta1 = math.pi/2 - the1
    
    if Entry[1] > Target[1]:
        theta2 = -the2
    else:
        theta2 = the2
        
    T1 = np.array([[np.cos(theta1),0,-np.sin(theta1)],[0,1,0],[np.sin(theta1),0,np.cos(theta1)]])
    T2 = np.array([[np.cos(theta2),np.sin(theta2),0],[-np.sin(theta2),np.cos(theta2),0],[0,0,1]])
    TMat = [T1,T2]
    
    return MajorAxisRadius,MinorAxisRadius,TMat,MidPoint,theta1,theta2

def CheckEllipsoidCollision(CLpoint1,CLpoint2,MidPoint,MajorAxisRadius,MinorAxisRadius,the1,the2):
    # print(CLpoint1,CLpoint2,MidPoint,MajorAxisRadius,MinorAxisRadius,the1,the2)
    stack = []
    IntersectionPoint1 = [0,0,0]
    IntersectionPoint2 = [0,0,0]
    #Components:
    l = (np.asarray(CLpoint2) - np.asarray(CLpoint1))
    p = np.asarray(CLpoint1)- np.asarray(MidPoint)
    CoeffMat = np.array([[1/np.square(MajorAxisRadius),0,0],[0,1/np.square(MinorAxisRadius),0],[0,0,1/np.square(MinorAxisRadius)]])
   
    T1 = np.array([[np.cos(the1),0,-np.sin(the1)],[0,1,0],[np.sin(the1),0,np.cos(the1)]])
    T2 = np.array([[np.cos(the2),np.sin(the2),0],[-np.sin(the2),np.cos(the2),0],[0,0,1]])
    
    R = np.dot(T2,T1)
    S = np.dot(np.dot(R,CoeffMat),np.transpose(R))
#     S = np.dot(np.dot(np.transpose(R),CoeffMat),R)

    #Quadratic Constants: 
    a = np.dot(np.dot(l,S),l)
    b = 2 * ( np.dot(np.dot(p,S),l))
    c = (np.dot(np.dot(p,S),p)) - 1
    
    #Solutions
    Solution1 = (-b + np.sqrt(np.square(b) - (4 * a * c))) / (2*a)
    Solution2 = (-b - np.sqrt(np.square(b) - (4 * a * c))) / (2*a)
    
    InOut1 = CheckInisideorOutsideEllipsoid(CLpoint1,MidPoint,S)
    InOut2 = CheckInisideorOutsideEllipsoid(CLpoint2,MidPoint,S)
    # print(Solution1,Solution2)
    if np.isnan(Solution1) and np.isnan(Solution2):
        stack.append(-1)
    else:
        IntersectionPoint1 = FindIntersectionPoints(CLpoint1[0],CLpoint1[1],CLpoint1[2],CLpoint2[0],CLpoint2[1],CLpoint2[2],Solution1)
        IntersectionPoint2 = FindIntersectionPoints(CLpoint1[0],CLpoint1[1],CLpoint1[2],CLpoint2[0],CLpoint2[1],CLpoint2[2],Solution2)

        VectorDistance =  EuclideanDistance(CLpoint1,CLpoint2)

        CheckDistance1 = EuclideanDistance(CLpoint1,IntersectionPoint1)
        CheckDistance2 = EuclideanDistance(CLpoint2,IntersectionPoint1)
        CheckDistance3 = EuclideanDistance(CLpoint1,IntersectionPoint2)
        CheckDistance4 = EuclideanDistance(CLpoint2,IntersectionPoint2)

        if CheckDistance1 <= VectorDistance and CheckDistance2 <= VectorDistance:
            stack.append(0)
        elif CheckDistance3 <=VectorDistance and CheckDistance4 <= VectorDistance:
            stack.append(0)
        else:
            stack.append(1)
            MinDistance = [CheckDistance1,CheckDistance2,CheckDistance3,CheckDistance4]
            # stack.append(np.min(MinDistance))    
            # print("IntersectionPoint1 : {}, IntersectionPoint2 : {}".format(IntersectionPoint1,IntersectionPoint2))
    
    if InOut1 <= 1 or InOut2 <= 1:
        stack = []
        stack.append(0)
    
    return stack


def CheckNeedleCollision(Entry_list,Target_list,X_Cods,Y_Cods,Z_Cods):

    Link_radius =[58,58,48,37.5,37.5]
    
    NoOfDivivsions = [15,15,20,30,30]

    for i in range(len(Entry_list) ):
        EllipsoidPoint1 = Entry_list[i]
        EllipsoidPoint2 = Target_list[i]
    
        r1,r2,TransMat,MP,the1,the2 = EllipsoidDetails(EllipsoidPoint1,EllipsoidPoint2)
        
           
        for cds in range(len(X_Cods)):
            Results = [] 
            current_frame = [X_Cods[cds],Y_Cods[cds],Z_Cods[cds]]
            for j in range(len(current_frame[0]) - 1):
                vec1 = np.array([current_frame[0][j],current_frame[1][j],current_frame[2][j]]) * 1000
                vec2 = np.array([current_frame[0][j+1],current_frame[1][j+1],current_frame[2][j+1]]) * 1000
        
                #Check Plane and Ellpisoid Details:
                Points1,Points2 = LineToCylinder(vec1,vec2,Link_radius[j],NoOfDivivsions[j])
                
                for EachLine in range(len(Points1)):
                    if len(Entry_list) != 1:
                        Results.append(CheckEllipsoidCollision(Points1[EachLine],Points2[EachLine],MP,r1,r2,the1,the2))
                        Results.append(CheckPlaneCollision(Points1[EachLine],Points2[EachLine]))
                    else :
                        Results.append(CheckPlaneCollision(Points1[EachLine],Points2[EachLine]))
                        
            Arr_results = np.asarray(Results)
            results = list(np.squeeze(Arr_results))
            # print(results)

            if 0 in results:
                return 0,current_frame
        
    return 1,current_frame

def CheckPathPointsCollisionV2(Entry_list,Target_list,pathpoints):

    
    for i in range(len(Entry_list) ):
        EllipsoidPoint1 = Entry_list[i]
        EllipsoidPoint2 = Target_list[i]
        
        r1,r2,TransMat,MP,the1,the2 = EllipsoidDetails(EllipsoidPoint1,EllipsoidPoint2)

        X = pathpoints[0]
        Y = pathpoints[1]
        Z = pathpoints[2]
        
        Results = []

        for j in range(len(pathpoints[0]) -1):
            point1 = [X[j],Y[j],Z[j]]
            point2 = [X[j+1],Y[j+1],Z[j+1]]
            
            Results.append(CheckEllipsoidCollision(point1,point2,MP,r1,r2,the1,the2))
            # print(Results)
        
        Arr_results = np.asarray(Results)
        results = list(np.squeeze(Arr_results))
        
        if 0 in results:
            return 0
        
    return 1
            

def CheckPlaneCollision(pt1,pt2,eps = 1e-6):
    stack = []
    height = 0
    p1 = [-800,-750,height]
    p2 = [-250,-750,height]
    p3 = [-800,750,height]
    p4 = [-250,750,height]
    x = [p1[0],p2[0],p3[0],p4[0]]
    y = [p1[1],p2[1],p3[1],p4[1]]
    z = [p1[2],p2[2],p3[2],p4[2]]
    xmin = min(x[0],x[1],x[2],x[3])
    xmax = max(x[0],x[1],x[2],x[3])
    ymin = min(y[0],y[1],y[2],y[3])
    ymax = max(y[0],y[1],y[2],y[3])
    
    vector = np.subtract(pt2,pt1)
    mag = np.linalg.norm(vector)
    threshold = mag
    unit_v = [vector[0]/mag,vector[1]/mag,vector[2]/mag]
    unit_v = np.asarray(unit_v)
    planeNormal = np.array([0, 0, 1])
    planePoint = np.array([x[1],y[1],z[1]])
    w = np.subtract(pt1,planePoint)
    N = -np.dot(planeNormal,w)
    D = np.dot(planeNormal,unit_v)
    # if abs(D) < eps:
    #     raise RuntimeError("no intersection")
            
    sI = N/D
    I=pt1+(sI*unit_v)
    
    if (I[0] <= xmax and I[0] >= xmin) and (I[1] <= ymax and I[1] >= ymin) and I[2] == z[1]:
        dist1 = np.linalg.norm(np.subtract(pt1,I))
        dist2 = np.linalg.norm(np.subtract(pt2,I))    
        if (threshold < dist1 or threshold < dist2):
            stack.append(1)
            return stack
        elif (threshold > dist1 or threshold > dist2):     
            stack.append(0)
            return stack
    else:
        if pt1[2] == pt2[2] == z[1]:
            stack.append(0)
            return stack
        else:
            stack.append(1)
            return stack


