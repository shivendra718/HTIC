

import urx
import math
from math import *
import math3d as m3d
import numpy as np
import time
import plotly.graph_objects as go
import socket 
import math
import plotly.io as pio
import os
# import tqdm

def ActualMovement(jv,a,v,r):
            
    a = "a = " + str(a)
    v = "v = " + str(v)
    r = "r = " + str(r)
    filename = "/home/shivendra/Visualization_Modules/Final_Testing_Path/example1.script"
    f = open (filename, "wb")
    f.write("def movement():\n".encode("utf8"))
    f.write("  while(True):\n".encode("utf8"))
    for i in range (len(jv)):
        message = str(list(jv[i]))
        if i == len(jv)-1:
            movej = "    "+"movej"+"("+message+","+a+","+v+")"+"\n"
            movej = movej.encode("utf8")
            f.write(movej)
        else:    
            movej = "    "+"movej"+"("+message+","+a+","+v+","+r+")"+"\n"
            movej = movej.encode("utf8")
            f.write(movej)
    f.write("    halt\n".encode("utf8"))
    f.write("    end\n".encode("utf8"))
    f.write("  end\n".encode("utf8"))
    f.write("end\n".encode("utf8"))
    f.close()


    HOST = "172.16.101.225"
    # HOST = "127.0.0.1"
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    f = open (filename, "rb") 
    l = f.read(1024)
    while (l):
        s.send(l)
        l = f.read(1024)
    s.close()
    
    return 0
