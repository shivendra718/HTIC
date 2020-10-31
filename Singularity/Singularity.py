import urx
import math
import numpy as np
import time

## The commented part in main is to get the continous getl and getj values

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

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def VecNorm(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    return norm

def Singularity(jv,step):
    stack = []
    jv_timestep = []
    dummy = []
    for i in range(len(jv)-1):
        mid = np.linspace(jv[i],jv[i+1],step)
        dummy.append(mid)
        for j in range(len(dummy[0])):
            jv_timestep.append(list(dummy[i][j]))
    for i in range(len(jv_timestep)):        
        Positions = JointLocations(jv_timestep[i])
    #     print(Positions)
        wristvec1 = np.asarray(Positions[3]) - np.asarray(Positions[2])
        wristvec2 = np.asarray(Positions[5]) - np.asarray(Positions[4])
        Resultatnt = np.cross(Vec2UnitVec( wristvec1), Vec2UnitVec( wristvec2))
        Res = VecNorm(Resultatnt)#wrist
        elbowvec1 = np.asarray(Positions[1]) - np.asarray(Positions[0])
        elbowvec2 = np.asarray(Positions[2]) - np.asarray(Positions[1])
        Resultatnt1 = np.cross(Vec2UnitVec( elbowvec1), Vec2UnitVec( elbowvec2))
        Res1 = VecNorm(Resultatnt1)#elbow
        
        p1 = Positions[0]
        p2 = [0,0.1333,0.1625]
#         p2 = [0,-Positions[3][1],p1[2]]
        vector1 = np.subtract(p2,p1)
        p = p1 + 0.5*(vector1)
        p = [p[0],p[1],p[2]+10]
        vec2 = np.subtract(p2,p)

        a,b,c = np.cross(vec2,vector1)
        d = a*p[0] + b*p[1] + c*p[2]
        d1 = a* Positions[2][0] + b*Positions[2][1] + c*Positions[2][2]
        d2 = a* Positions[3][0] + b*Positions[3][1] + c*Positions[3][2]
        if (d1 <= d+0.2 and d1 >= d-0.2) and (d2 <= d+0.2 and d2 >= d-0.2): 
            print("Shoulder")
            retval = 1
            stack.append(retval)
        elif Res>=0 and Res<=0.2:
            print("Wrist")
            retval = 1
            stack.append(retval)
        elif Res1>=0 and Res1<=0.2:
            print("Elbow")
            retval = 1
            stack.append(retval)
        else:
            print("NO SINGULARITY")
            retval = 0
            stack.append(retval)
    if np.any(stack) == 1:
        return 1
    else: 
        return 0

if __name__ == '__main__':
    # robo = urx.Robot("172.16.101.225")

    # current_value_l = []
    # current_value_j = []
    # while robo.is_program_running():
    #     c = robo.getl()
    #     d = robo.getj()
    #     current_value_l.append(c)
    #     current_value_j.append(d)
    #     time.sleep(1)
    # print(current_value_l)
    # print(current_value_j)
    # print(len(current_value_j))

    # Singularity(current_value_j)


    Singularity([[-0.005850617085592091, -1.5645685985549171, 1.5609763304339808, 0.0032141643711547374, 1.564843773841858, 3.141602039337158], [-0.09828502336610967, -1.467383624320366, 1.407058064137594, 0.055456681842468214, 1.4727669954299927, 3.1417346000671387], [-0.21900493303407842, -1.340588541035988, 1.2059343496905726, 0.12356845914807124, 1.3521817922592163, 3.141890048980713], [-0.3396723906146448, -1.213582531814911, 1.0047758261310022, 0.19151656209912105, 1.231461524963379, 3.1420702934265137], [-0.4603779951678675, -1.0866501492312928, 0.8034470717059534, 0.2597578006931762, 1.1109284162521362, 3.1422266960144043], [-0.5810535589801233, -0.9596792024425049, 0.6024130026446741, 0.3280644851871948, 0.9903329014778137, 3.142394542694092], [-0.7020762602435511, -0.8324709695628663, 0.40057355562318975, 0.39616112291302485, 0.8697267770767212, 3.1425867080688477], [-0.8229916731463831, -0.7055670779994507, 0.19945985475649053, 0.46403996526684566, 0.749167263507843, 3.142730712890625], [-0.9435527960406702, -0.5785462421229859, -0.0016330626094713807, 0.532353325481079, 0.6286482214927673, 3.1428868770599365]],5)
