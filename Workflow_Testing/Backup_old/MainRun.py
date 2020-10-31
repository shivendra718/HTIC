########################################################################

# READ ME
# This code contains intial setup for workflow testing
# Modules addressed are 1. Path Planning and recorrection, 2. Collision avoidance and recorrection.

########################################################################


# Python Required Packages
import urx
import math
import numpy as np
import time
import plotly.graph_objects as go


#Functions from other files (Listed in the same directory)
from modules import *
from PlotsFunction import *
from PathPlanningModule import *
from CollisionFunction import *


#Print options:
np.set_printoptions(suppress=True)

#System usgae variables : 
InputType = 0 # 1 for user feed, 0 for hard code


#Staring Main Thread : 

if __name__ == '__main__':

    # Variable Declarations
    EntryList = []
    TargetList = []
    counter = 0
    signal = 0
    RemoveVar = 0
    Ep_Var = 0
    

    #Input from user : 

    while(signal == 0):

        if counter != 0:
            DeleteGraphPoints( Plot_Figure, pathpoints, ETpoints)
        
        if (InputType == 1):
            Ep_Var = 1
            print("Enter Entry Values : ")
            val = input()
            Entry = list(val)
            if len(Entry) != 3:
                print("Please entry proper x,y,z coordinates")
                val = input()
                Entry = list(val)
            EntryList.append(Entry)

            print("Enter Target Values : ")
            val = input()
            Target = list(val)
            if len(Target) != 3:
                print("Please entry proper x,y,z coordinates")
                val = input()
                Target = list(val)
            TargetList.append(Target)

        else:
            
            counter = 1
            #To be filled by user
            Entry = [-520,-410,200]
            Target = [-600,-370,10]

            # Mention atleast 2 needles
            EntryList = [[-650,-400,200]]
            TargetList = [[-600,-350,5]] 
            
            signal = 1
        

        #Initialize Simulated Robot:
        Plot_Figure = RobotBaseStructure()
        Plot_Figure,ETpoints = TraceEntTarpoints(Plot_Figure,Entry,Target) #Plot Current Entry and Target


        #PathPlanning Module:

        #Variables:
        
        Initial_point = [-491.8,-133.3,487.8]
        ICD = 0 #Initiail centre distance
        Final_point = Entry
        WST = 300 #Workspace threshold
        
        #Planner : 
        print("Started the thread :  Path planning")
        PathCoordinates, Plot_Figure, pathpoints = PathPlanner(Initial_point, ICD, Final_point, EntryList, TargetList, WST, Plot_Figure)

        if PathCoordinates != 0:
            print("Started the thread : Collision Avoidance")

            # Computing change in Orientation
            I_orient,F_orient = GetInitialFinalOrientation(Entry,Target)
            Timesteps = len(PathCoordinates[0])
            TimeStepOrientation = RetriveTimeStepOrientation(I_orient,F_orient,Timesteps)

            #Ellpsoid Plotting
            if counter != 0:
                for i in range(len(EntryList) - Ep_Var):
                    r1,r2,TransMat,MP,the1,the2 = EllipsoidDetails(EntryList[i],TargetList[i])
                    Plot_Figure = DisplayEllipsoid(Plot_Figure,MP,TransMat,r1,r2)
            ShowCurrentPlot(Plot_Figure)
            
        # Collision Avoidance Module
            X_Cods,Y_Cods,Z_Cods,CollidingFrame = RecorrectOrientation(PathCoordinates,TimeStepOrientation,EntryList,TargetList)
            
            if X_Cods != 0 and Y_Cods != 0 and Z_Cods != 0:
                # Collision Frame:
                Plot_Figure,pts = FrameVisual(Plot_Figure,CollidingFrame)
                ShowCurrentPlot(Plot_Figure)
                
                l = list(Plot_Figure.data)
                for i in range(len(pts)):
                    l.remove(pts[i])
                Plot_Figure.data = tuple(l)
                
                #MovementFrame
                Plot_Figure = plot_frames(Plot_Figure,X_Cods,Y_Cods,Z_Cods)
                print("Showing Final graph")
                ShowCurrentPlot(Plot_Figure)
            else: 
                RemoveVar = 1
        else: 
            RemoveVar = 1
            ShowCurrentPlot(Plot_Figure)
        
        if RemoveVar == 1 and InputType == 1:
            EntryList.remove(Entry)
            TargetList.remove(Target)

        counter += 1




# Things to look at

# 1. Plotfunction : FrameVisuals
# 2. CollisionFunction : CheckNeedleCollision