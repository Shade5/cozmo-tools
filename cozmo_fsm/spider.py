import cv2
import socket
import pickle
import threading
from pdb import set_trace
from time import sleep

class Server(threading.Thread):
    def __init__(self, robot, port=1800):
        threading.Thread.__init__(self)
        self.port = port
        self.socket = None #not running until startServer is called
        self.robot= robot

    def run(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setblocking(True) #lets select work properly I think
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True) #enables easy server restart
        self.socket.bind(("",self.port)) #binds to any address the computer can be accessed by
        self.threads =[]

        for i in range(100):
            self.socket.listen(5)   # Now wait for client connection.
            c, addr = self.socket.accept()    # Establish connection with client.
            print('Got connection from', addr)
            self.threads.append(Spawn(i, "Client"+str(i), c, self.robot))
            self.threads[i].start()

class Spawn(threading.Thread):
    def __init__(self, threadID, name, client, robot):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.c = client
        self.robot = robot
    def run(self):
        while(True):
            self.c.sendall(pickle.dumps(self.robot.world.pcam.cams)+b'end')
            data = b''
            while True:
                data += self.c.recv(1024)
                if data[-3:]==b'end':
                    break

            cams = pickle.loads(data[:-3])

            for key, value in cams.items():
                if key in self.robot.world.pcam.cams:
                    self.robot.world.pcam.cams[key][value.cap] = value

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
        print("Connected.")
        self.start()

    def run(self):
        while(True):
            data = b''
            while True:
                data += self.socket.recv(1024)
                if data[-3:]==b'end':
                    break

            cams = pickle.loads(data[:-3])

            for key, value in cams.items():
                if key in self.robot.world.pcam.cams:
                    self.robot.world.pcam.cams[key][value.cap] = value
            self.socket.sendall(pickle.dumps(self.robot.world.pcam.cams)+b'end')



