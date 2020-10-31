import plotly.graph_objects as go
import numpy as np
import urx
import math
import plotly.express as px
from decimal import *
from numpy import *
from scipy.linalg import norm

from CollisionFunction import *
from modules import *
from PlotFunctions import *

def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")

#Rodrigues form :
def rodrot(vec,axis,theta):
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return angle

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

# ********PATHS***********

def SmoothPathPoints(a,b,h,pt1):
    
    global distance
    distance = []
    c = [(a[0]+b[0])/2,(a[1]+b[1])/2]
    vector = np.subtract(b,a)
    dist = norm(vector) 
    div = dist*6
    tmin = 0
    tmax = np.pi
    cmin = a[0]
    cmax = b[0]
    bmin = a[1]
    bmax = b[1]
    C = linspace(cmin, cmax, div)
    t = linspace(tmin, tmax, div)
    B = linspace(bmin, bmax, div)
    y = (h)*(sin((t)) ) + B


    for i in range(0,len(C)-1):
        for j in range(0,len(pt1)):
            if C[i] <= pt1[j][0] and C[i+1] >= pt1[j][0]:
                magni = np.sqrt((C[i]-pt1[j][0])**2 + (B[i]-pt1[j][1])**2)
                magni1 = np.sqrt((C[i]-C[i+1])**2 + (y[i]-y[i+1])**2)
                p1 = [C[i],y[i]]
                p2 = [C[i+1],y[i+1]]
                v = np.subtract(p2,p1)
                scale = (magni/magni1)
                d = scale + magni 
                l2 = p1 + (d*(v))
                if pt1[j] == b:
                    l2 = b
                dist = np.sqrt((pt1[j][0]-l2[0])**2 + (pt1[j][1]-l2[1])**2)
                distance.append(dist)

    return distance



def SubPathPoints(VecMag,SF):
    # SF = 100
    CenDist =  SF
    Factors = [0,0.15,0.3,0.45,0.6,0.75,0.85,0.95]
    hstack1 = list(np.array(Factors) * CenDist)
    hstack2 = list(np.flip(hstack1))
    hstack1.append(CenDist)
    GroundDistances = hstack1 + hstack2

    Div_factor2 = float(Decimal(VecMag) / Decimal(16))
    istack = list(np.arange(0,VecMag,Div_factor2)) 
    istack.append(VecMag)
    istack = list(np.flip(istack))
    
    return GroundDistances,istack

def SubPathPoints2(VecMag,SF):

    # SF = 100 
    #Needed Changes:
    # Div_factor2 = float(Decimal(VecMag) / Decimal(16))
    Div_factor2 = int(VecMag / 17)
    istack = list(np.arange(0,VecMag,Div_factor2)) 
    istack.append(VecMag)
  
    istack = list(np.flip(istack))

    Out = []
    In = []

    for i in istack:
        In.append(i)
        In.append(0)
        Out.append(In)
        In = []

    start = [0,0]
    end = [VecMag,0]
    GroundDistances = SmoothPathPoints(start,end,SF,Out)
    # print(len(Out))
    # print(len(GroundDistances))

    return GroundDistances,istack


def GeneratePathPoints(InitialPoint,FinalPoint,flag,theta,CentreDistance):
    
    points = []
    PlanarPoint = [FinalPoint[0],FinalPoint[1],InitialPoint[2]]
    PlanarVector = np.asarray(PlanarPoint) - np.asarray(InitialPoint)
    
    PathVector = np.asarray(FinalPoint) - np.asarray(InitialPoint)
    LCAxisVectorY = np.cross(PathVector,PlanarVector)
    LCAxisVectorZ = np.cross(LCAxisVectorY,PlanarVector)

    E2Distance = np.sqrt( np.square(FinalPoint[0]- InitialPoint[0]) + np.square(FinalPoint[1]- InitialPoint[1]) + np.square(FinalPoint[2]- InitialPoint[2]))
    ActualAngle = AnglebtwnVecs(PathVector,PlanarVector)

    UaxisX = Vec2UnitVec(PlanarVector)
    UaxisY = Vec2UnitVec(LCAxisVectorY)
    UaxisZ = Vec2UnitVec(LCAxisVectorZ)
    
    # if flag == 1:
    #     UaxisZ = rodrot(UaxisZ,UaxisX,theta)
    #     UaxisY = np.cross(UaxisX,UaxisZ)
        

    RotationVector1 = UaxisX
    AxisVector = UaxisY
    LCX = rodrot(RotationVector1 * E2Distance,AxisVector,-ActualAngle)
    RotationVector2 = UaxisZ
    LCY = rodrot(RotationVector2 * E2Distance,AxisVector,-ActualAngle)
    
    CuLCX = Vec2UnitVec(LCX)

    if flag == 1:
        LCY = rodrot(LCY,CuLCX,theta)

    LCZ = np.cross(LCX,LCY)


    # PathVectorMD = InitialPoint + (Vec2UnitVec(LCX) * E2Distance/2)
    PathGenVector = Vec2UnitVec(LCY)
    PathGenAxis = Vec2UnitVec(LCZ)
    
    #Choose smoothing or nor:
    # Gpoints,ipoints = SubPathPoints2( E2Distance,CentreDistance)
    Gpoints,ipoints = SubPathPoints( E2Distance,CentreDistance)
    
    T_vector = Vec2UnitVec(PathVector)
    
    for i in range(len(ipoints)):
        # RotVec = rodrot(PathGenVector,PathGenAxis,0)
    
        Rp = InitialPoint + (T_vector  * ipoints[i]) + (PathGenVector * Gpoints[i])
        points.append(Rp)

    aapoints = np.asarray(points)
    xpoints = list(aapoints[:,0])
    ypoints = list(aapoints[:,1])
    zpoints = list(aapoints[:,2])
    
    return xpoints,ypoints,zpoints


# def Recorrectpath(PathS,PathE,IO,FO,EL,TL,Plot_Figure):

#     for inc in np.arange(0.1,math.pi,0.1):
#         Incre = inc
#         for loop2 in range(2):
#             xpoints,ypoints,zpoints = GeneratePathPoints(PathS,PathE,1,Incre)
#             PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
#             Timesteps = len(PathCoordinates[0])
#             Timesteps = len(PathCoordinates[0])
#             TimeStepOrientation = RetriveTimeStepOrientation(IO,FO,Timesteps)
#             X_Cods,Y_Cods,Z_Cods = RetriveIKSolutions(PathCoordinates,TimeStepOrientation)
#             Plot_Figure,e_ = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
            
#             stat = CheckNeedleCollision(EL,TL,X_Cods,Y_Cods,Z_Cods)

#             if stat == 1:
#                 print("Collision Free Path found")
#                 return X_Cods,Y_Cods,Z_Cods
#             Incre = -Incre
#     Plot_Figure.show()
#     return [0,0,0]

def RecorrectOrientation(Paths,Orientations,EL,TL,Plot_Figure):
    
    
    theta = 0.1
    soln = [0,2,5,7]
    for ii in range(len(soln)): 
        stat_ = 0
        solution = soln[ii]
        while (stat_ == 0):

            for i in range(2):
                
                
                X_Cods,Y_Cods,Z_Cods = RetriveIKSolutions(Paths,Orientations,1,theta,solution)
                Status = CheckNeedleCollision(EL,TL,X_Cods,Y_Cods,Z_Cods)
            
                if Status == 1:
                    stat_ = 1
                    print("found a solution")
                    return X_Cods,Y_Cods,Z_Cods

                theta = -theta
                print("Failed at angle {}".format(theta))
            
            theta = theta + 0.1

            

            if theta > math.pi :
                stat_ = 1

    print("Orientation not working")
    return [0,0,0]




def CheckPathSingularity(DataX,DataY,DataZ):

    status = 0
    for i in range(len(DataX)):

        cfX = DataX[i]
        cfY = DataY[i]
        cfZ = DataZ[i]

        pt1 = [cfX[0],cfY[0],cfZ[0]]
        pt2 = [cfX[1],cfY[1],cfZ[1]]
        pt3 = [cfX[2],cfY[2],cfZ[2]]
        pt4 = [cfX[3],cfY[3],cfZ[3]]
        pt5 = [cfX[4],cfY[4],cfZ[4]]
        pt6 = [cfX[5],cfY[5],cfZ[5]]

        Wrist = np.cross(pt4,pt6)
        print(Wrist)
        Shoulder = np.cross(pt2,pt6)
        Elbow = np.cross(pt2,pt4)

        if (Wrist[0] >= -0.1 and Wrist[1] >= -0.1 and Wrist[2]>=-0.1) and (Wrist[0]<=0.1 and Wrist[1]<=0.1 and Wrist[2]<=0.1):
            status = 1
            print("Wrist singularity",i)
            # break    
        if (Shoulder[0] >= -0.1 and Shoulder[1] >= -0.1 and Shoulder[2]>=-0.1) and (Shoulder[0]<=0.1 and Shoulder[1]<=0.1 and Shoulder[2]<=0.1):
            status = 1
            # break
            print("Shoulder singularity",i)
        if (Elbow[0] >= -0.1 and Elbow[1] >= -0.1 and Elbow[2]>=-0.1) and (Elbow[0]<=0.1 and Elbow[1]<=0.1 and Elbow[2]<=0.1):
            status = 1
            # break
            print("Elbow singularity",i)


    return status,i