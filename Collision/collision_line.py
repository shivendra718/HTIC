def collision_line(p1,p2,step):
    
    #newpts_array,r,step = Cylinder([2,1,1],[5,6,5],1.6,7)

    
    data = go.Scatter3d(x=[p1[0],p2[0]], y=[p1[1],p2[1]], z=[p1[2],p2[2]])
    fig.add_trace(data)
    #fig.show()
    
    
    for i in range(0,step):
        dist_v = pdist*unit_v
        #print(dist_v)
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

    #    print(x_1,x_2)

        sol1 = [p1[0]*(1-x_2) + x_2*p2[0],
            p1[1]*(1-x_2) + x_2*p2[1],
            p1[2]*(1-x_2) + x_2*p2[2]]

        sol2 = [p1[0]*(1-x_1) + x_1*p2[0],
            p1[1]*(1-x_1) + x_1*p2[1],
            p1[2]*(1-x_1) + x_1*p2[2]]


        if (discriminant < 0):
             print(0)
        elif((x_2>1 or x_2<0) and (x_1>1 or x_1<0)):
             print(0)
        elif (not(x_2 > 1 or x_2 < 0) and (x_1 > 1 or x_1 < 0)):
            print("Intersection point:",sol1)
            dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
            dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
            if dist1<dist2:
                print(dist1)
            else: 
                print(dist2)
        elif ((x_2 > 1 or x_2 < 0) and not(x_1 > 1 or x_1 < 0)):
            print("Intersection point:",sol2)
            dist1 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
            dist2 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
            if dist1<dist2:
                print(dist1)
            else: 
                print(dist2)
        else:
            print("Intersection point:",sol1,sol2)
            dist1 = sqrt((sol1[0]-p1[0])**2 + (sol1[1]-p1[1])**2 + (sol1[2]-p1[2])**2)
            dist2 = sqrt((sol1[0]-p2[0])**2 + (sol1[1]-p2[1])**2 + (sol1[2]-p2[2])**2)
            dist3 = sqrt((sol2[0]-p1[0])**2 + (sol2[1]-p1[1])**2 + (sol2[2]-p1[2])**2)
            dist4 = sqrt((sol2[0]-p2[0])**2 + (sol2[1]-p2[1])**2 + (sol2[2]-p2[2])**2)
            print(dist1,dist2,dist3,dist4)



Cylinder([2,1,1],[5,6,5],1.6,7)
Cylinder([2,2,1],[2,2,8],1.6,7)
#collision_line([3,3,3],[7,7,7])

print(start_end)
print(step)
fig.show()