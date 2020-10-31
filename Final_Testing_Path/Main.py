#This code contains fixes of Initial and Final Orientation, Joint angles convertion(Joint 2,3 : 2pi and Joint6:pi)
import urx
import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *
import plotly.graph_objects as go
from modules import *

from PathPlanningModule import *
from PlotsFunction import *
from ActualRobotMovement import *

#Robot Connect: 
robo = urx.Robot("172.16.101.225")

#Plot Function:
def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()

if __name__ == '__main__':

    #initialize Simulated robot
    Plot_Figure = RobotBaseStructure()
    
    #Entry Target : 
    Entry = [-649.7162204069298,-137.95809657884417,-23.874797606461887]
    Target = [-643.67993354, -184.13957269, -175.51543535]

    EntryList = [[520,-150,200],[300,-250,200],[550,-145,250]]
    TargetList = [[381,-150,0],[420,-265,0],[350,-130,0]]    

    Initial_point = [-580.7463793216489,-132.92468862753978,435.7082672380777]


    #Initial and Final Orientation:
    Initial_Orientation = np.array([[ 0.21581449, -0., -0.97643438],
                                  [-0.97643438, -0. , -0.21581449],
                                  [ 0. ,  1. , -0]])

    EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 
    x,y,z = OrientMatix(EM,TM)
    Orientation = [x,y,z]
    Final_Orientation = np.transpose(Orientation)

    Orient = RetriveTimeStepOrientation(Initial_Orientation,Final_Orientation,17)

    #Plot Current Entry and Target
    Plot_Figure,ETpoints = TraceEntTarpoints(Plot_Figure,Entry,Target) 
    
    
    #Path Retrival
    ICD = 50 #Initiail centre distance
    Final_point = Entry
    WST = 300 #Workspace threshold
    #Planner : 
    print("Started the thread :  Path planning")
    PathCoordinates, Plot_Figure, pathpoints = PathPlanner(Initial_point, ICD, Final_point, EntryList, TargetList, WST, Plot_Figure)
    print(pathpoints)


    

    #ShowPlot
    ShowCurrentPlot(Plot_Figure)
    #Main
    Joint_Values = []
    print("IK Path")
    for i in range(len(PathCoordinates[0])):

        PathPoint = [PathCoordinates[0][i],PathCoordinates[1][i],PathCoordinates[2][i]]
        POSI,ORIEE = PositionRetrival(PathPoint,Orient[i])
        if i == 1:
            ORIEE = np.array([[ 0.00128245,  0.00062998, -0.99999898],
                                [ 0.99999772, -0.00171009,  0.00128137],
                                [-0.00170928, -0.99999834, -0.00063217]])
        ang = solUR5ik(POSI,ORIEE)
        print(ang[0])
        Joint_Values.append(ang[0])

    print("Modified Path")
    Modified_JV = PathContinuityVerification(Joint_Values)
    
    print("Make Movement ?")
    inp = int(input())
    if inp == 1:
        ActualMovement(Modified_JV,0.15,0.15,0.02)
