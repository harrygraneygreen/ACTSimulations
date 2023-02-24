import os, sys, socket, re, json, random
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

#sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
#sumoCmd = [sumoBinary, "-c", "longhighway.sumo.cfg", "--start"]

import traci
import sumolib
from sumolib import checkBinary
import sumolib.net
from sumolib.net import readNet
from sumolib.net import Net
from sumolib.net import NetReader
from sumolib.net import lane
from sumolib.net import edge
from sumolib.net import node
from sumolib.net import connection
from sumolib.net import roundabout
from sumolib.net.edge import Edge

##################### Lead Traj ########################
dt = 1/10
duration = 1500
m2m = 1/2.2369
v18na = np.loadtxt('na_speed_18mph.txt')
a18na = np.diff(v18na)/0.1
a18na = np.append(0,a18na)
for i in range(len(a18na)):
    if a18na[i]>5:
        a18na[i] = 5
    elif a18na[i]<-6:
        a18na[i] = -6

#plt.plot(a18na)
v18new = np.zeros(a18na.shape)
v18new[0] = v18na[0]
for i in range(1,len(v18new)):
    v18new[i] = v18new[i-1] + 0.1*a18na[i]*1.3
v18new = v18new/m2m-13.2
for i in range(1,len(v18new)):
    if v18new[i]<0:
        v18new[i] = 0
v0 = v18new

x = np.linspace(0, v0.shape[0], v0.shape[0])
x1 = np.linspace(0, v0.shape[0], v0.shape[0]*6)
from scipy.interpolate import interp1d
vinterp = interp1d(x, v0)
v01 = vinterp(x1)
########################## Parameters of platoon ##########################
#N_lane_1, N_lane_2, N_lane_3 = 5, 5, 2
N_lane_1, N_lane_2, N_lane_3 = 10, 0, 0

N_VEHICLES = N_lane_1 + N_lane_2 + N_lane_3

lead_v = v01
lead_a = np.diff(lead_v)/dt
lead_a = np.append(0, lead_a)

# used to randomly color the vehicles
random.seed(1)
step = 0

#max_iter = len(lead_a)*1
max_iter = 5000
#max_iter = 2
#print(max_iter)

position = np.zeros((max_iter+1, N_VEHICLES))
pos_ego = np.zeros((max_iter+1,))
vel_ego = np.zeros((max_iter+1,))
speed = np.zeros((max_iter+1, N_VEHICLES))
accel = np.zeros((max_iter+1, N_VEHICLES))
light = np.zeros((max_iter+1, N_VEHICLES))


######CAR FOLLOWING MODELS############


def EIDM(spacing, v, v0, s0=3, a0=1.8, b0=2.8, T=1, sigma=4, Kf=1, Kg=0.6, *cv_arg):

    cv_arg = cv_arg[0]
    CAV_count = len(cv_arg)//2
    temp_v, temp_a = 0, 0
    alpha, beta = np.zeros((1,CAV_count)), np.zeros((1,CAV_count))
    if CAV_count > 0:
        for i in range(CAV_count):
            alpha[i] = 0.3/(0.3+np.exp(i))
            beta[i] = 0.3/(0.3+np.exp(i))
            temp_v += alpha[i]*cv_arg[2*i]
            temp_a += beta[i]*cv_arg[2*i+1]
        #s_star = s0 + v*T - v*alpha[0]*cv_arg[0]/(2*np.sqrt(a0*b0))
        s_star = s0 + v*T - v*cv_arg[0]/(2*np.sqrt(a0*b0))
    else:
        print('Error because no CAV speed and acceleration input!')
    a_free = a0*(1-(v/v0)**sigma)
    a_int = -a0*(s_star/spacing)**2
    acc = a_free + a_int

    return Kf*acc + Kg*(temp_v + temp_a)

    ####Used to get list of omegas(cars that are in commncation range)

    def in_communicationrange(self, candidate):
        leadv_list=[]
        trash = []
        #print("length of candidate: " + str(len(candidate)))
        for i in range(len(candidate)):
            if 0 < (candidate[i].x - self.x) and (candidate[i].x - self.x)<= self.communication_range:
                leadv_list.append(candidate[i])
            else:
                trash.append(candidate[i])
        if trash and trash[-1].id > leadv_list[0].id:
            print("asffasfdf")

        return leadv_list

def within_cr(self, other, step):
    if (abs(position[step, self] - position[step, other]) < 200):
        return True
    return False

#another function to return the array of vn's and xn's NO

# def ACH(spacing, v, v0, x0, omegas, *cv_arg):
#     rel_v = cv_arg[0]
#     rel_acc = cv_arg[1]
#     cv_arg = cv_arg[0]
#     CAV_count = len(cv_arg)//2
#     temp_v, temp_x = 0, 0
#     #alpha, beta = np.zeros((1,CAV_count)), np.zeros((1,CAV_count))

#     omegas = in_communicationrange()

#     if CAV_count > 0:
#         for i in range(CAV_count):
#             temp_v += omegas[i] * alpha * (vel[i] - v0)
#             temp_x += omegas[i] * beta * (pos[i] - x0)


#     vn = temp_v + temp_x

#     return vn




def modelA(space, step, v, rel_v, rel_acc, index, laneBound):
    alpha = .001

    k1 = .5
    k2 = .5
    #currPos = position[step, index]
    currPos = (traci.vehicle.getPosition("%d" % index))[1]
    currSpeed = traci.vehicle.getSpeed("%d" % index)
    inRange = []
    for i in range(index - laneBound - 1):
        #if (abs(position[step, index - i] - currPos) < 200):
           # inRange.append(i)
        if (abs((traci.vehicle.getPosition("%d" % i)[1]) - currPos) < 200) :
            inRange.append(i)
    if len(inRange) == 0:
        return k1*rel_v + k2*space
    else:
        index = random.randint(0, len(inRange) - 1)
        selected = inRange[index]
        #s2 = position[step,selected + laneBound] - position[step,index]
        s2 = traci.vehicle.getPosition("%d" % (selected + laneBound))[1] - currPos
        #v2 = speed[step,selected + laneBound] - v
        v2 = traci.vehicle.getSpeed("%d" % (selected + laneBound)) - currSpeed
        a2 = accel[step,selected + laneBound] - accel[step,index]
        return alpha*rel_v + alpha*space + alpha*v2 + alpha*s2

def model1Car(space, step, v, rel_v, rel_acc, index, laneBound):

    alpha = .5

    k1 = .5
    k2 = .5
    #currPos = position[step, index]
    currPos = (traci.vehicle.getPosition("%d" % index))[1]
    currSpeed = traci.vehicle.getSpeed("%d" % index)
    return alpha*rel_v + alpha*space

def model2Cars(space, step, v, rel_v, rel_acc, index, laneBound):
    alpha = .5

    k1 = .5
    k2 = .1
    #currPos = position[step, index]
    currPos = (traci.vehicle.getPosition("%d" % index))[1]
    currSpeed = traci.vehicle.getSpeed("%d" % index)
    inRange = []
    for i in range(index - laneBound - 1):
        inRange.append(i)
    if index < 2:
        return alpha*rel_v + alpha*space
    else:
        alpha = .25
        s2 = traci.vehicle.getPosition("%d" % (index-2 + laneBound))[1] - currPos
        #v2 = speed[step,selected + laneBound] - v
        v2 = traci.vehicle.getSpeed("%d" % (index-2 + laneBound)) - currSpeed
        a2 = accel[step,i-2 + laneBound] - accel[step,index]
        return k1*rel_v + k1*space + k2*v2 + k2*s2


def model3Cars(space, step, v, rel_v, rel_acc, index, laneBound):
    alpha = .5

    k1 = .6
    k2 = .2
    k3 = .12
    #currPos = position[step, index]
    currPos = (traci.vehicle.getPosition("%d" % index))[1]
    currSpeed = traci.vehicle.getSpeed("%d" % index)
    inRange = []
    for i in range(index - laneBound - 1):
        inRange.append(i)
    if index < 2:
        return alpha*rel_v + alpha*space
    elif index < 3:
        alpha = .25
        s2 = traci.vehicle.getPosition("%d" % (index-2 + laneBound))[1] - currPos
        #v2 = speed[step,selected + laneBound] - v
        v2 = traci.vehicle.getSpeed("%d" % (index-2 + laneBound)) - currSpeed
        a2 = accel[step,i-2 + laneBound] - accel[step,index]
        return k1*rel_v + k1*space + k2*v2 + k2*s2
    else:

        s2 = traci.vehicle.getPosition("%d" % (index-2 + laneBound))[1] - currPos
        #v2 = speed[step,selected + laneBound] - v
        v2 = traci.vehicle.getSpeed("%d" % (index-2 + laneBound)) - currSpeed
        a2 = accel[step,i-2 + laneBound] - accel[step,index]

        s3 = traci.vehicle.getPosition("%d" % (index-3 + laneBound))[1] - currPos
        #v2 = speed[step,selected + laneBound] - v
        v3 = traci.vehicle.getSpeed("%d" % (index-3 + laneBound)) - currSpeed
        a3 = accel[step,i-3 + laneBound] - accel[step,index]

        return k1*rel_v + k1*space + k2*v2 + k2*s2 + k3*v3 + k3*s3



def model5Cars(space, step, v, rel_v, rel_acc, index, laneBound):


    currPos = (traci.vehicle.getPosition("%d" % index))[1]
    currSpeed = traci.vehicle.getSpeed("%d" % index)
    #inRange = []
    kvals = [.05,.05, .1, .3, .5]
    alphas = [0,0,0,0,0]
    relPos = []
    relVel = []
    if (index >= 5) :
        for i in range(5):
            j = index - 5 + i
            #if (abs(position[step, index - i] - currPos) < 200):
               # inRange.append(i)
            if (abs((traci.vehicle.getPosition("%d" % j)[1]) - currPos) < 300):
                #inRange.append(i)
                alphas[i] = 1
            relPos[i] = traci.vehicle.getPosition("%d" % (index - 5 + i + laneBound))[1] - currPos
            relVel[i] = traci.vehicle.getSpeed("%d" % (index - 5 + i + laneBound)) - currSpeed

     else :
        for i in range(index):
            if (abs((traci.vehicle.getPosition("%d" % i)[1]) - currPos) < 300):
                alphas[i+(5-index)] = 1
            relPos[i+(5-index)] = traci.vehicle.getPosition("%d" % (i + laneBound))[1] - currPos
            relVel[i+(5-index)] = traci.vehicle.getSpeed("%d" % (i + laneBound)) - currSpeed

    tempArr = [0,0,0,0,0]
    for i in range (5):
        tempArr[i] = alphas[i] * kvals[i]

    tempSum = sum(tempArr)
    multFactor = 1.0 / tempSum

    for i in range (5):
        tempArr[i] *= multFactor
    sum = 0;
    for i in range(5):
        sum += tempArr[i]*relPos[i] + tempArr[i]*relVel[i]
    return sum




def sim_setting3(pos, space, v, rel_v, alpha, i, step, lane):
    front = alpha*rel_v + alpha*space
    while (lane < i-1):
        if (within_cr(i, lane, step)):
            break
        else:
            lane += 1
    if (lane >= i-1):
        return accel[step, i]
    else:
        rand_index = random.randint(lane, i-2)
        new_pos = position[step, rand_index]
        new_vel = speed[step, rand_index]
        rand_vehicle = alpha*(new_vel - v) + alpha*(new_pos - pos)
    return front + rand_vehicle


def acc_linear(h, v, space, rel_v, rel_a, ks, kv, ka):
    return ks*(space-h*v) + kv*rel_v + ka*rel_a

#SS
s0=2.5
a0=1
b0=2
T=2.6
sigma=4
Kf=1
Kg=0

length = 5
spd = lead_v[0]
v0 = 33
dis = (s0+spd*T)/np.sqrt(1-(spd/v0)**sigma)


#SU
h = 1
ks, kv, ka = 5, 5, 2
dis = spd*h + 3
#########################################################################

graph = sumolib.net.readNet('LLC.net.xml', withInternal=True) #internal edge are edges inside interseciton or connections
vertex = graph.getNodes()
edge = graph.getEdges(withInternal=True)
#print('edge length:')
#print(len(edge))

sumoCmd = [sumolib.checkBinary('sumo-gui'), '-c', 'LLC_SUMO_no.sumo.cfg']
traci.start(sumoCmd)
step = 0

traci.route.add(routeID = 'route1', edges = ['-17.0.00'])
#traci.vehicle.add('ego', "route")
traci.vehicle.add('ego', "route1", departPos=str(200), departSpeed=str(spd), departLane = str(0))
traci.vehicle.setColor('ego', color=(255, 0, 0, 255))
traci.vehicle.setLaneChangeMode('ego', 256)
#print(traci.vehicle.getRoadID('ego'))
#print(traci.vehicle.getPosition('ego'))
#print(traci.vehicle.getLanePosition('ego'))
#UDP_IP = "192.168.0.11"
UDP_IP = "192.168.0.181"
UDP_PORT = 23333

# serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# serverSock.bind((UDP_IP, UDP_PORT))

# data, address = serverSock.recvfrom(1024*6)
# data = data.decode('utf-8').split("$")[0]
# y, x, z, vx, vy = data.split(";")


x, y, z = '814.82','50', '.5'
# X_offset, Y_offset = 814.72, 0.55  #882.504, 846.15
#x, y, z = X_offset+float(x.replace("\x00", "")), Y_offset+float(y.replace("\x00", "")), float(z.replace("\x00", ""))*180/3.14159265
#vx, vy = float(vx.replace("\x00", "")), float(vy.replace("\x00", ""))
vx,vy=0,0
eEdge = traci.vehicle.getRoadID('ego')
#print(eEdge)
ePos = traci.vehicle.getPosition('ego')
traci.vehicle.moveToXY('ego', '', 0, x, y, angle=z, keepRoute=2)
traci.simulationStep()

def dis_ego(vID_tmp):
	posi = traci.vehicle.getPosition(vID_tmp)
	dis = ((ePos[0] - posi[0])**2 + (ePos[1] - posi[1])**2)**(1/2)
	return dis

while step<=max_iter:

	# data, address = serverSock.recvfrom(1024*6)
	# data = data.decode('utf-8').split("$")[0]
	# y, x, z, vx, vy = data.split(";")
	#x, y = '6448.97', '-6645.57'
	# X_offset, Y_offset = 814.72, 0.55  #882.504, 846.15
	# x, y, z = X_offset+float(x.replace("\x00", "")), Y_offset+float(y.replace("\x00", "")), float(z.replace("\x00", ""))*180/3.14159265
    # vx, vy = float(vx.replace("\x00", "")), float(vy.replace("\x00", ""))
    x, y, z = '814.82','50', '.5'
    vx,vy=0,0
    eEdge = traci.vehicle.getRoadID('ego')
    ePos = traci.vehicle.getPosition('ego')
    traci.vehicle.moveToXY('ego', '', 0, x, y, angle=z, keepRoute=2)
    traci.vehicle.setSpeed('ego', 1)
    eLanePos = traci.vehicle.getLanePosition('ego')
	#pint(eLanePos)
    pos_ego[step] = eLanePos
    vel_ego[step] = np.sqrt(vx**2+vy**2)

    if step==0:
		#eLanePos = 10
        for i in range(N_VEHICLES):
            if i < N_lane_1:
                position[0, i] = eLanePos + (N_lane_1-i)*dis + 5 - dis*N_lane_1//2
                speed[0, i] = spd
                accel[0, i] = 0

                vid = "%d" % i
                traci.vehicle.add(vid, "route1", departPos=str(position[0, i]), departSpeed=str(spd), departLane = str(0), typeID="vtypeauto")
                traci.vehicle.setColor(vid, (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255))
                traci.vehicle.setLaneChangeMode(vid, 256)
            elif i >= N_lane_1 and i < N_lane_1 + N_lane_2:
                position[0, i] = eLanePos + (N_lane_1+N_lane_2-i)*dis + 10 - dis*N_lane_2//2
                speed[0, i] = spd
                accel[0, i] = 0

                vid = "%d" % i
                traci.vehicle.add(vid, "route1", departPos=str(position[0, i]), departSpeed=str(spd), departLane = str(1), typeID="vtypeauto")
                traci.vehicle.setColor(vid, (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255))
                traci.vehicle.setLaneChangeMode(vid, 256)

            else:
                position[0, i] = eLanePos + (N_lane_1+N_lane_2+N_lane_3-i)*dis + 8
                speed[0, i] = spd
                accel[0, i] = 0

                vid = "%d" % i
                traci.vehicle.add(vid, "route1", departPos=str(position[0, i]), departSpeed=str(spd), departLane = str(2), typeID="vtypeauto")
                traci.vehicle.setColor(vid, (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255))
                traci.vehicle.setLaneChangeMode(vid, 256)

            traci.gui.trackVehicle("View #0", "%d" % i)
            traci.gui.setZoom("View #0", 3000)

    elif step>=1:
        if True:
            for i in range(N_VEHICLES):
                if i == 0 or i == N_lane_1 or i == N_lane_1+N_lane_2:
                    #traci.vehicle.setSpeed("v.%d" % i, lead_v[step//10])
                    traci.vehicle.setSpeed("%d" % i, lead_v[step])
                    #traci.vehicle.setSpeed("%d" % i, speed[step-1,i]+0.001)
                    #traci.vehicle.setSpeed("%d" % i, speed[step,i] + lead_a[step//1]*dt)
                    #if lead_a[step//1]<0:
                    #    traci.vehicle.setSignals("%d" % i, 3)


                else:
                    if i < N_lane_1:
                        #space = traci.vehicle.getLanePosition("v.%d" % (i-1)) - traci.vehicle.getLanePosition("v.%d" % i)
                        position[step,i] = traci.vehicle.getPosition(str(i))[1]
                        space = position[step,i-1] - position[step,i]
                        speed[step,i] = traci.vehicle.getSpeed(str(i))
                        accel[step,i] = traci.vehicle.getAcceleration(str(i))
                        # position[step,i] = traci.vehicle.getPosition("%d" % i)[1]
                        # speed[step,i] = traci.vehicle.getSpeed("%d" % i)
                        # accel[step,i] = traci.vehicle.getAcceleration("%d" % i)

                        #v = traci.vehicle.getSpeed("v.%d" % i)
                        v = speed[step,i]
                        #rel_v = traci.vehicle.getSpeed("v.%d" % (i-1)) - traci.vehicle.getSpeed("v.%d" % i)
                        rel_v = speed[step,i-1] - speed[step,i]
                        #rel_acc = traci.vehicle.getAcceleration("v.%d" % (i-1))- traci.vehicle.getAcceleration("v.%d" % i)
                        rel_acc = accel[step,i-1] - accel[step,i]
                        #acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = modelA(space, step, v, rel_v, rel_acc, i, laneBound)
                        #acceleration = sim_setting3(position[step,i], space, v, rel_v, 0.6, i, step, 0)
                        # acceleration = model1Car(space, step, v, rel_v, rel_acc, i, 0)
                        acceleration = model3Cars(space, step, v, rel_v, rel_acc, i, 0)
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)
                    elif i > N_lane_1 and i < N_lane_1 + N_lane_2:
                        position[step,i] = traci.vehicle.getPosition(str(i))[1]
                        space = position[step,i-1] - position[step,i]
                        speed[step,i] = traci.vehicle.getSpeed(str(i))
                        accel[step,i] = traci.vehicle.getAcceleration(str(i))
                        # position[step,i] = traci.vehicle.getPosition(str(i))[1]
                        # speed[step,i] = traci.vehicle.getSpeed(str(i))
                        # accel[step,i] = traci.vehicle.getAcceleration(str(i))

                        v = speed[step,i]
                        rel_v = speed[step,i-1] - speed[step,i]
                        rel_acc = accel[step,i-1] - accel[step,i]
                        #acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = modelA(space, step, v, rel_v, rel_acc, i, laneBound)
                        #acceleration = sim_setting3(position[step,i], space, v, rel_v, 0.6, i, step, N_lane_1)
                        # acceleration = model1Car(space, step, v, rel_v, rel_acc, i, 0)
                        acceleration = model3Cars(space, step, v, rel_v, rel_acc, i, 0)
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)
                    else:
                        position[step,i] = traci.vehicle.getPosition(str(i))[1]
                        space = position[step,i-1] - position[step,i]
                        speed[step,i] = traci.vehicle.getSpeed(str(i))
                        accel[step,i] = traci.vehicle.getAcceleration(str(i))
                        # position[step,i] = traci.vehicle.getPosition(str(i))[1]
                        # speed[step,i] = traci.vehicle.getSpeed(str(i))
                        # accel[step,i] = traci.vehicle.getAcceleration(str(i))

                        v = speed[step,i]
                        rel_v = speed[step,i-1] - speed[step,i]
                        rel_acc = accel[step,i-1] - accel[step,i]
                        #acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = modelA(space, step, v, rel_v, rel_acc, i, laneBound)
                        #acceleration = sim_setting3(position[step,i], space, v, rel_v, 0.6, i, step, N_lane_1 + N_lane_2)
                        # acceleration = model1Car(space, step, v, rel_v, rel_acc, i, 0)
                        acceleration = model3Cars(space, step, v, rel_v, rel_acc, i, 0)
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)
                    new_spd = speed[step,i] + acceleration*dt
                    traci.vehicle.setSpeed("%d" % i, speed[step,i] + acceleration*dt)
                    if (i == 2):
                        print(new_spd)
                    drag_decel = -0.3   #0.12 + (0.25*(speed[step,i] + acceleration*dt)**2)/1750
                    if traci.vehicle.getAcceleration("%d" % i) < drag_decel:
                        light[step,i] = 3
                        traci.vehicle.setSignals("%d" % i, 3)
                    else:
                        light[step,i] = 0
                        traci.vehicle.setSignals("%d" % i, 0)
                    # if i==N_VEHICLES-1:
                    #     print(step, light[step,i], traci.vehicle.getSignals("%d" % i), traci.vehicle.getAcceleration("%d" % i), traci.vehicle.getSpeed("%d" % i))
    #print(traci.vehicle.getPosition('ego'))
    #print(traci.vehicle.getLanePosition('ego'))
    #print(step, traci.vehicle.getSignals("%d" % 21), )
    #Message=''
    #Message =  "0" + "," + "0" + "," + str(step)+ "," + Message
    for k in range(N_VEHICLES):
        vID = str(k)

        Position = traci.vehicle.getPosition(vID)
        sig = int(light[step,k])      #sig = traci.vehicle.getSignals(vID)
        vel = traci.vehicle.getSpeed(vID)

        #if (Position[0]-x)**2 + (Position[1]-y)**2 > 90000:
        #		continue
        #Message += str(vID)+","

        #xx = "{0:.3f}".format(position[0]-x)
        #xx = "{0:.3f}".format(Position[0]-X_offset)

        #yy = "{0:.3f}".format(position[1]-y)
        #yy = "{0:.3f}".format(Position[1]-Y_offset)

        #Message += xx + "," + yy +","
        #print(vID,Position[0],Position[1])
        #print(vID,xx,yy)
        #angle = traci.vehicle.getAngle(vID)
        #angle = "{0:.3f}".format(angle)
        #Message += angle + ","

        #Message +=  str(sig) + "," + str(vel) + ","

    #Message = Message[:-1]
    #print(repr(Message))
    #serverSock.sendto(Message.encode('utf-8'), (address[0], 23334))
    pltcp = 4500
    if step == pltcp:
        plt.plot(position[11:pltcp, 3])
        pos3 = np.asarray(position[11:pltcp, 3])
        pd.DataFrame(pos3).to_csv('pos3')
        plt.plot(position[11:pltcp, 9])
        pos9 = np.asarray(position[11:pltcp, 9])
        pd.DataFrame(pos9).to_csv('pos9')
        #plt.xlim(500)
        plt.show()
        plt.plot(speed[11:pltcp, 3])
        spd3 = np.asarray(speed[11:pltcp, 3])
        pd.DataFrame(spd3).to_csv('spd3')
        plt.plot(speed[11:pltcp, 9])
        spd9 = np.asarray(speed[11:pltcp, 9])
        pd.DataFrame(spd9).to_csv('spd9')
        #plt.xlim(500)
        plt.show()
        plt.plot(accel[11:pltcp, 3])
        accel3 = np.asarray(accel[11:pltcp, 3])
        pd.DataFrame(accel3).to_csv('accel3')
        plt.plot(accel[11:pltcp, 9])
        accel9 = np.asarray(accel[11:pltcp, 9])
        pd.DataFrame(accel9).to_csv('accel9')
        #plt.xlim(500)
        plt.show()
    step += 1

    for i in range(N_VEHICLES):
        position[step,i] = traci.vehicle.getLanePosition("%d" % i)
        speed[step,i] = traci.vehicle.getSpeed("%d" % i)
        accel[step,i] = traci.vehicle.getAcceleration("%d" % i)
        #light[step,i] = traci.vehicle.getSignals("%d" % i)

    #print(step-1, light[step-1,11], traci.vehicle.getSignals("%d" % 11), traci.vehicle.getAcceleration("%d" % 11), traci.vehicle.getSpeed("%d" % 11))
    #print(traci.vehicle.getSpeed(str(N_lane_0 + N_lane_1)))
    traci.simulationStep()
    if step >= max_iter-1:
        # lets plot the position and speeds for each vehicle here
        #time = np.array()
        #speedVals =

        np.savetxt('./run_pos_na_cong_su.txt', position)
        np.savetxt('./run_vel_cong_su.txt', speed)
        np.savetxt('./run_accel_cong_su.txt', accel)
        break


traci.close()
