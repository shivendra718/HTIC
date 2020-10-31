#import Packages:
import urx
import math
import math3d as m3d
import numpy as np
import time
import quaternion
import datetime
np.set_printoptions(suppress=True)
import plotly.graph_objects as go

#Import other Pre-deffined functions:
from modules import * 
from PathFuctions import *
from PlotFunctions import *
from CollisionFunction import *  


def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")

#Main Run:

if __name__ == '__main__':

    EntryList = []
    TargetList = []
    name = "needle"
    needlearray = []
    counter = 0
    signal = 0

    Plot_Figure = RobotBaseStructure()

    # while (signal == 0):
        
    if counter != 0:
        DeleteGraphPoints(Plot_Figure,pathpoints,ETpoints)


    Entry = [-650,-133,200]
    Target = [-401,-141,0]

    EntryList = [[-560,-150,450],[-530,10,390],[-580,-140,420],[-520,-130,460]]
    TargetList = [[-561,-151,0],[-531,11,0],[-501,-151,0],[-301,-180,0]]
    
    # Initialize virtual robot:
    Plot_Figure,ETpoints = TraceEntTarpoints(Plot_Figure,Entry,Target)
    
    #Generating Path:
    Initial_point = [-491.8,-133.3,487.8]
    Initial_Centre_distance = 10
    Final_point = Entry
    xpoints,ypoints,zpoints = GeneratePathPoints(Initial_point,Final_point,0,0,Initial_Centre_distance)
    PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
    Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)

    threshold = 1000
    PPstatus = CheckPathPointsCollisionV2(EntryList,TargetList,PathCoordinates)
    # PPstatus = 1
    if PPstatus == 0:
        print("Path points are colliding, Recorrection under progress")
        inc_distance = Initial_Centre_distance
        loop_begins = 1
        
        while (loop_begins == 1):
            loop_begins_1 = 1
            inc_distance += 10
            print(inc_distance)
            xpoints,ypoints,zpoints = GeneratePathPoints(Initial_point,Final_point,0,0,inc_distance)
            PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
            PPstatus1 = CheckPathPointsCollisionV2(EntryList,TargetList,PathCoordinates)
           
            if PPstatus1 == 1:
                loop_begins = 0
                print("Path rectified")
                print(inc_distance)
                Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
                break 
            
            val = 0
            while (loop_begins_1 == 1):
                
                val = val + 0.1
                xpoints,ypoints,zpoints = GeneratePathPoints(Initial_point,Final_point,1,val,inc_distance)
                PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
                PPstatus1 = CheckPathPointsCollisionV2(EntryList,TargetList,PathCoordinates)
                if PPstatus1 == 1:
                    loop_begins_1 = 0
                    print("Path rectified")
                    print(inc_distance)
                    print("Angle,",val)
                    Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
                    loop_begins = 0
                    break
                
                if val > 2*math.pi:
                    loop_begins_1 = 0
                    print("Path Cannot be found")
                
    else:
        print("Path points free to go")
        Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
    
    #Sampling Orientation
    I_orient,F_orient = GetInitialFinalOrientation(Entry,Target)
    # F_orient = np.array([[ -0.0113645,  0.2779514, -0.9605279],[0.0800259, -0.9572559, -0.2779514],[ -0.9967280, -0.0800259, -0.0113645 ]])

    if len(EntryList) == 1:

        #Sampling Positions and Orientations:

        #Check for F_orient is null:
        Timesteps = len(PathCoordinates[0])
        TimeStepOrientation = RetriveTimeStepOrientation(I_orient,F_orient,Timesteps)
        X_Cods,Y_Cods,Z_Cods = RetriveIKSolutions(PathCoordinates,TimeStepOrientation,0,0,0)
        Plot_Figure = plot_frames(Plot_Figure,X_Cods,Y_Cods,Z_Cods)
        print("Showing current graph")
        ShowCurrentPlot(Plot_Figure)
        time.sleep(1)
        
        
    else:
        
        for i in range(len(EntryList)):
            r1,r2,TransMat,MP,the1,the2 = EllipsoidDetails(EntryList[i],TargetList[i])
            Plot_Figure = DisplayEllipsoid(Plot_Figure,MP,TransMat,r1,r2)

        print("Showing current graph")
        ShowCurrentPlot(Plot_Figure)
        time.sleep(1)

    #     # Processing
        Timesteps = len(PathCoordinates[0])
        TimeStepOrientation = RetriveTimeStepOrientation(I_orient,F_orient,Timesteps)
        X_Cods,Y_Cods,Z_Cods = RetriveIKSolutions(PathCoordinates,TimeStepOrientation,0,0,0)


        print("Check collision, Yes : 1, No : 0")
        user_input1 = input()
        if user_input1 == 1:
            Status = CheckNeedleCollision(EntryList,TargetList,X_Cods,Y_Cods,Z_Cods)
            if Status == 1:
                print("Collision free zone, Ready to plan")
            else:   
                print("**Collision detected**, Recorrecting...")
                # X_Cods,Y_Cods,Z_Cods = Recorrectpath(Initial_point,Final_point,I_orient,F_orient,EntryList,TargetList,Plot_Figure)
                X_Cods,Y_Cods,Z_Cods = RecorrectOrientation(PathCoordinates,TimeStepOrientation,EntryList,TargetList,Plot_Figure)

        if X_Cods == 0 and Z_Cods == 0:
            print("Cant Plan for a needle, try other")
            # EntryList.remove(Entry)
            # TargetList.remove(Target)
            # counter = counter - 1
            
        else:
            Plot_Figure = plot_frames(Plot_Figure,X_Cods,Y_Cods,Z_Cods)
            print("Showing current graph")
            ShowCurrentPlot(Plot_Figure)
       
       
       
    