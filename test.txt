import os, sys, socket, re, json, random
import numpy as np
import matplotlib.pyplot as plt

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
dt = 1/60
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
N_lane_0, N_lane_1, N_lane_2 = 5, 5, 2

N_VEHICLES = N_lane_0 + N_lane_1 + N_lane_2

lead_v = v01
lead_a = np.diff(lead_v)/dt
lead_a = np.append(0, lead_a)

# used to randomly color the vehicles
random.seed(1)
step = 0

max_iter = len(lead_a)*1
#max_iter = 2
#print(max_iter)

position = np.zeros((max_iter+1, N_VEHICLES))
pos_ego = np.zeros((max_iter+1,))
vel_ego = np.zeros((max_iter+1,))
speed = np.zeros((max_iter+1, N_VEHICLES))
accel = np.zeros((max_iter+1, N_VEHICLES))
light = np.zeros((max_iter+1, N_VEHICLES))


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
traci.vehicle.add('ego', "route1", departPos=str(200), departSpeed=str(spd), departLane = str(2), typeID="passenger")
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


x, y, z = '820', '12075','0'
# X_offset, Y_offset = 814.72, 0.55  #882.504, 846.15
#x, y, z = X_offset+float(x.replace("\x00", "")), Y_offset+float(y.replace("\x00", "")), float(z.replace("\x00", ""))*180/3.14159265
#vx, vy = float(vx.replace("\x00", "")), float(vy.replace("\x00", ""))
vx,vy=0,0
eEdge = traci.vehicle.getRoadID('ego')
print(eEdge)
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
    x, y, z = '820', '12075','0'
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
            if i < N_lane_0:
                position[0, i] = eLanePos + (N_lane_0-i)*dis + 5 - dis*N_lane_0//2
                speed[0, i] = spd
                accel[0, i] = 0
                
                vid = "%d" % i
                traci.vehicle.add(vid, "route1", departPos=str(position[0, i]), departSpeed=str(spd), departLane = str(0), typeID="vtypeauto")
                traci.vehicle.setColor(vid, (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255))
                traci.vehicle.setLaneChangeMode(vid, 256)
            elif i >= N_lane_0 and i < N_lane_0 + N_lane_1:
                position[0, i] = eLanePos + (N_lane_0+N_lane_1-i)*dis + 10 - dis*N_lane_1//2
                speed[0, i] = spd
                accel[0, i] = 0
                
                vid = "%d" % i
                traci.vehicle.add(vid, "route1", departPos=str(position[0, i]), departSpeed=str(spd), departLane = str(1), typeID="vtypeauto")
                traci.vehicle.setColor(vid, (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255))
                traci.vehicle.setLaneChangeMode(vid, 256)
            
            else:
                position[0, i] = eLanePos + (N_lane_0+N_lane_1+N_lane_2-i)*dis + 8
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
                if i == 0 or i == N_lane_0 or i == N_lane_0+N_lane_1:
                    #traci.vehicle.setSpeed("v.%d" % i, lead_v[step//10])
                    traci.vehicle.setSpeed("%d" % i, speed[step,i] + lead_a[step//1]*dt)
                    if lead_a[step//1]<0:
                        traci.vehicle.setSignals("%d" % i, 3)
                else:
                    if i < N_lane_0:
                        #space = traci.vehicle.getLanePosition("v.%d" % (i-1)) - traci.vehicle.getLanePosition("v.%d" % i)
                        space = position[step,i-1] - position[step,i]
                        #v = traci.vehicle.getSpeed("v.%d" % i)
                        v = speed[step,i]
                        #rel_v = traci.vehicle.getSpeed("v.%d" % (i-1)) - traci.vehicle.getSpeed("v.%d" % i)
                        rel_v = speed[step,i-1] - speed[step,i]
                        #rel_acc = traci.vehicle.getAcceleration("v.%d" % (i-1))- traci.vehicle.getAcceleration("v.%d" % i)
                        rel_acc = accel[step,i-1] - accel[step,i]
                        acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)                        
                    elif i > N_lane_0 and i < N_lane_0 + N_lane_1:
                        space = position[step,i-1] - position[step,i]
                        v = speed[step,i]
                        rel_v = speed[step,i-1] - speed[step,i]
                        rel_acc = accel[step,i-1] - accel[step,i]
                        acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)
                    else:
                        space = position[step,i-1] - position[step,i]
                        v = speed[step,i]
                        rel_v = speed[step,i-1] - speed[step,i]
                        rel_acc = accel[step,i-1] - accel[step,i]
                        acceleration = EIDM(space, v, v0, s0, a0, b0, T, sigma, Kf, Kg, (rel_v,rel_acc))
                        #acceleration = acc_linear(h, v, space, rel_v, rel_acc, ks, kv, ka)
                    traci.vehicle.setSpeed("%d" % i, speed[step,i] + acceleration*dt)
                    drag_decel = -0.3   #0.12 + (0.25*(speed[step,i] + acceleration*dt)**2)/1750
                    if traci.vehicle.getAcceleration("%d" % i) < drag_decel:
                        light[step,i] = 3
                        traci.vehicle.setSignals("%d" % i, 3)
                    else:
                        light[step,i] = 0                            
                        traci.vehicle.setSignals("%d" % i, 0)
                    if i==N_VEHICLES-1:
                        print(step, light[step,i], traci.vehicle.getSignals("%d" % i), traci.vehicle.getAcceleration("%d" % i), traci.vehicle.getSpeed("%d" % i))
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
        np.savetxt('./run_pos_na_cong_su.txt', position)
        np.savetxt('./run_vel_cong_su.txt', speed)
        np.savetxt('./run_accel_cong_su.txt', accel)
        break

traci.close()