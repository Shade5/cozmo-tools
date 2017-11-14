import cv2
import socket
import pickle
import threading
from pdb import set_trace
from time import sleep
from numpy import inf, arctan2, pi, cos, sin
from .worldmap import RobotGhostObj

class Server(threading.Thread):
    def __init__(self, robot, port=1800):
        threading.Thread.__init__(self)
        self.port = port
        self.socket = None #not running until startServer is called
        self.robot= robot
        self.landmark_pool = {}
        self.landmark_pool[self.robot.aruco_id]={}
        self.poses = {}
        self.fusion = Fusion(self.robot)

    def run(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setblocking(True) #lets select work properly I think
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True) #enables easy server restart
        self.socket.bind(("",self.port)) #binds to any address the computer can be accessed by
        self.threads =[]
        print("Server started")
        self.fusion.start()
        self.robot.world.is_server = True

        for i in range(100):
            self.socket.listen(5)   # Now wait for client connection.
            c, addr = self.socket.accept()    # Establish connection with client.
            print('Got connection from', addr)
            self.threads.append(Spawn(i, c, self.robot))
            self.threads[i].start()

class Spawn(threading.Thread):
    def __init__(self, threadID, client, robot):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.c = client
        self.robot = robot
        self.c.sendall(pickle.dumps("Hello"))
        self.aruco_id = int(pickle.loads(self.c.recv(1024)))
        self.name = "Client-"+str(self.aruco_id)
        self.robot.world.server.landmark_pool[self.aruco_id]={}
        print("Started thread for",self.name)

    def run(self):
        while(True):
            self.c.sendall(pickle.dumps(self.robot.world.pcam.pool)+b'end')
            data = b''
            while True:
                data += self.c.recv(1024)
                if data[-3:]==b'end':
                    break

            cams, landmarks, pose = pickle.loads(data[:-3])
            for key, value in cams.items():
                if key in self.robot.world.pcam.pool:
                    self.robot.world.pcam.pool[key].update(value)
                else:
                    self.robot.world.pcam.pool[key]=value

            self.robot.world.server.landmark_pool[self.aruco_id].update(landmarks)
            self.robot.world.server.poses[self.aruco_id] = pose

class Fusion(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
        self.aruco_id = self.robot.aruco_id
        self.accurate = {}
        self.transforms = {}

    def run(self):
        while(True):
            self.robot.world.server.landmark_pool[self.aruco_id].update({k:self.robot.world.particle_filter.sensor_model.landmarks[k] for k in [x for x in self.robot.world.particle_filter.sensor_model.landmarks.keys() if isinstance(x,str) and "Video" in x]})

            # Choose accurate camera
            for key1, value1 in self.robot.world.server.landmark_pool.items():
                for key2, value2 in self.robot.world.server.landmark_pool.items():
                    if key1 == key2:
                        continue
                    for cap, lan in value1.items():
                        if cap in value2:
                            varsum = lan[2].sum()+value2[cap][2].sum()
                            if varsum < self.accurate.get((key1,key2),(inf,None))[0]:
                                self.accurate[(key1,key2)] = (varsum,cap)

            # Find transform
            for key, value in self.accurate.items():
                x1,y1 = self.robot.world.server.landmark_pool[key[0]][value[1]][0]
                h1,p1 = self.robot.world.server.landmark_pool[key[0]][value[1]][1]
                x2,y2 = self.robot.world.server.landmark_pool[key[1]][value[1]][0]
                h2,p2 = self.robot.world.server.landmark_pool[key[1]][value[1]][1]

                theta_t = (p1 - p2)
                x_t = x2 - ( x1*cos(theta_t) + y1*sin(theta_t))
                y_t = y2 - (-x1*sin(theta_t) + y1*cos(theta_t))

                self.transforms[key] = (x_t, y_t, theta_t)

            self.update_ghosts()
            sleep(0.01)

    def update_ghosts(self):
        for key, value in self.transforms.items():
            if key[1] == 90:
                x_t, y_t, theta_t = value
                x, y, theta = self.robot.world.server.poses[key[0]]
                x2 =  x*cos(theta_t) + y*sin(theta_t) + x_t 
                y2 = -x*sin(theta_t) + y*cos(theta_t) + y_t 
                self.robot.world.world_map.objects["Ghost-"+str(key[0])]=RobotGhostObj(cozmo_id=key[0], x=x2, y=y2, z=0, theta=theta-theta_t, is_visible=True,)

class Client(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.port = None
        self.socket = None #not running until startClient is called
        self.ipaddr = None
        self.robot= robot

    def startClient(self,ipaddr="",port=1800):
        self.port = port
        self.ipaddr = ipaddr
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True)
        print("Attempting to connect to %s at port %d" % (ipaddr,port))
        self.socket.connect((ipaddr,port))
        data = pickle.loads(self.socket.recv(1024))
        print("Connected.")
        self.socket.sendall(pickle.dumps(self.robot.aruco_id))
        self.robot.world.is_server = False
        self.start()

    def run(self):
        while(True):
            data = b''
            while True:
                data += self.socket.recv(1024)
                if data[-3:]==b'end':
                    break

            self.robot.world.pcam.pool = pickle.loads(data[:-3])
            self.socket.sendall(pickle.dumps([self.robot.world.pcam.cams,
                {k:self.robot.world.particle_filter.sensor_model.landmarks[k] for k in [x for x in self.robot.world.particle_filter.sensor_model.landmarks.keys() if isinstance(x,str) and "Video" in x]},
                self.robot.world.particle_filter.pose])+b'end')



