import cv2
import socket
import pickle
import threading
import random
from numpy import arctan2, sqrt, pi
from cozmo_fsm import *
from pdb import set_trace

class LocateCam(StateNode):
    """ Locates Camera1."""
    def __init__(self, camera_number=1, camera_matrix=None, distCoeffs=None):
        self.camera_number = camera_number
        self.owner = socket.gethostname()
        self.camera_matrix = np.matrix([[1148.00,       -3,    641.0],
                         [0.000000,   1145.0,    371.0],
                         [0.000000, 0.000000, 1.000000]])
        self.distCoeffs = np.array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])
        super().__init__()


    def getframe(self):
        for i in range(5):
            self.cap.grab()
        ret, self.frame = self.cap.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        return self.gray

    def getcorners(self):
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def rotationMatrixToEulerAngles(self, R) :
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = arctan2(R[2,1] , R[2,2])
            y = arctan2(-R[2,0], sy)
            z = arctan2(R[1,0], R[0,0])
        else :
            x = arctan2(-R[1,2], R[1,1])
            y = arctan2(-R[2,0], sy)
            z = 0
     
        return np.array([x, y, z])

    def start(self, event=None):
        super().start(event)

        if self.camera_number not in self.parent.caplist:
            self.parent.caplist[self.camera_number] = cv2.VideoCapture(self.camera_number)
            self.parent.caplist[self.camera_number].set(3,4000)
            self.parent.caplist[self.camera_number].set(4,4000)

        self.cap = self.parent.caplist[self.camera_number]
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)

        print("Camera"+str(self.camera_number)+" resolution", self.camera_width,'x', self.camera_height)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        tries = 0

        while(True):
            tries += 1
            print('Try',tries)

            if tries >5:
                print("Cannot find cozmo from Camera"+str(self.camera_number%10))
                self.post_completion()
                return

            self.getframe()
            self.corners, self.ids = self.getcorners()
            print(self.ids)

            if type(self.ids) is np.ndarray:
                for id in range(len(self.ids)):
                    if self.ids[id] > 5:
                        continue
                    vecs = aruco.estimatePoseSingleMarkers(self.corners[id], 50, self.camera_matrix, self.distCoeffs)
                    rvecs, tvecs = vecs[0], vecs[1]
                    rotationm, jcob = cv2.Rodrigues(rvecs[0])
                    transformed = np.matrix(rotationm).T*(-np.matrix(tvecs).T)
                    print('Calibrated on',self.ids[id][0])
                    break
                break


        camera_x = -transformed[1,0]
        camera_y = transformed[0,0]
        camera_z = transformed[2,0]
        euler_angles = self.rotationMatrixToEulerAngles(-rotationm)
        print("Camera" +str(self.camera_number)+ " at: ",camera_x, camera_y, camera_z, euler_angles)
        self.robot.world.world_map.objects['Cam-'+self.owner+'-'+str(self.camera_number)] = CameraObj(random.randint(0,255), camera_x, camera_y, camera_z, euler_angles[0], euler_angles[2], rotationm, tvecs, self.ids[id][0])
        self.parent.supreme_cozmo_id = self.ids[id][0]
        print("World Coordinate frame is the current frame of Cozmo",self.parent.supreme_cozmo_id)
        self.post_completion()


class WallSpec():
    def __init__(self, length=100, height=210, door_width=75, door_height=105,
                 markers={}, doorways=[]):
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.markers = markers
        self.doorways = doorways
        ids = list(markers.keys())
        self.id = min(ids)
        global wall_marker_dict
        for id in ids:
            wall_marker_dict[id] = self


class ProcessImage(StateNode):
    def __init__(self, camera_number=1, camera_matrix=None, distCoeffs=None):
        self.camera_number = camera_number
        self.owner = socket.gethostname()
        self.camera_matrix = np.matrix([[1148.00,       -3,    641.0],
                         [0.000000,   1145.0,    371.0],
                         [0.000000, 0.000000, 1.000000]])
        self.distCoeffs = np.array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])
        super().__init__()
        self.cap = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters =  cv2.aruco.DetectorParameters_create()
        self.flag=0
        self.seen=dict(dict())
        super().__init__()

    def uncertainity(self, corners):
        return(abs(np.mean(corners[0][0][:,1])/self.camera_height -0.5) + abs(np.mean(corners[0][0][:,0])/self.camera_width -0.5))

    def rotationMatrixToEulerAngles(self, R) :
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = arctan2(R[2,1] , R[2,2])
            y = arctan2(-R[2,0], sy)
            z = arctan2(R[1,0], R[0,0])
        else :
            x = arctan2(-R[1,2], R[1,1])
            y = arctan2(-R[2,0], sy)
            z = 0
     
        return np.array([x, y, z])

    def infer_wall(self, id,markers):
        world_points = []
        image_points = []
        marker_size=50 #mm
        wall_spec = wall_marker_dict.get(markers[0][0],None)
        if wall_spec is None: return  # spurious marker
        for key, value in markers:
            (s, (cx, cy)) = wall_spec.markers[key]

            world_points.append((cx-marker_size/2, cy+marker_size/2,0))
            world_points.append((cx+marker_size/2, cy+marker_size/2,0))
            world_points.append((cx+marker_size/2, cy-marker_size/2,0))
            world_points.append((cx-marker_size/2, cy-marker_size/2,0))
            
            image_points.append(value[0])
            image_points.append(value[1])
            image_points.append(value[2])
            image_points.append(value[3])
            
        (success,rvecs, tvecs) = cv2.solvePnP(np.array(world_points), np.array(image_points), self.camera_matrix, self.distCoeffs)
        rotationm, jcob = cv2.Rodrigues(rvecs)
        transformed = np.matrix(rotationm).T*(-np.matrix(tvecs))
        
        return rvecs,tvecs, wall_spec
        
    
    
    def generate_walls_from_markers(self, corners, ids):
        seen_markers = dict()
        # Distribute markers to wall ids
        for i in range(len(ids)):
            id = ids[i][0]
            wall_spec = wall_marker_dict.get(id,None)
            if wall_spec is None: continue  # marker not part of a known wall
            wall_id = wall_spec.id
            markers = seen_markers.get(wall_id, list())
            markers.append((id,corners[i][0]))
            seen_markers[wall_id] = markers
            # Now infer the walls from the markers
        Cam = self.robot.world.world_map.objects['Cam-'+self.owner+'-'+str(self.camera_number)]
        for (id,markers) in seen_markers.items():
            rvecs,tvecs2, wall_spec  = self.infer_wall(id,markers)
            rotationm2, jcob = cv2.Rodrigues(rvecs)
            tvecs1 = Cam.tvecs[0][0]
            rotationm1 = Cam.rotm
            location = np.matrix(rotationm1).T*(np.matrix(tvecs2)-np.matrix(tvecs1).T)
            wall_orient = self.rotationMatrixToEulerAngles(-rotationm1)[2] - self.rotationMatrixToEulerAngles(-rotationm2)[2]
            wall_y = location[0,0] + wall_spec.length*cos(wall_orient)/2
            wall_x = -location[1,0] - wall_spec.length*sin(wall_orient)/2
            

            self.robot.world.world_map.objects[wall_spec.id+10] = WallObj(id=wall_spec.id, x=wall_x, y=wall_y, theta=wall_orient,
                           length=wall_spec.length, height=wall_spec.height,
                           door_height=wall_spec.door_height, doorways = wall_spec.doorways, ghost=True )

    def start(self, event=None):
        if self.camera_number not in self.parent.caplist:
            self.parent.caplist[self.camera_number] = cv2.VideoCapture(self.camera_number)
            self.parent.caplist[self.camera_number].set(3,4000)

        self.cap = self.parent.caplist[self.camera_number]
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)
        super().start(event)
        #Update Ghost
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        self.corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        for key in self.seen:
            self.seen[key]=False
        Cam = self.robot.world.world_map.objects['Cam-'+self.owner+'-'+str(self.camera_number)]
        if type(ids) is np.ndarray:
            vecs = aruco.estimatePoseSingleMarkers(self.corners, 50, self.camera_matrix, self.distCoeffs)
            rvecs, tvecs2 = vecs[0], vecs[1]
            for id in range(len(ids)):
                rotationm2, jcob = cv2.Rodrigues(rvecs[id])
                if ids[id] > 5:
                    continue
                try:
                    tvecs1 = Cam.tvecs[0][0]
                    rotationm1 = Cam.rotm
                    location = np.matrix(rotationm1).T*(np.matrix(tvecs2[id][0]).T-np.matrix(tvecs1).T)
                    Y = location[0,0]
                    X = -location[1,0]
                    phi = self.rotationMatrixToEulerAngles(-rotationm1)[2] - self.rotationMatrixToEulerAngles(-rotationm2)[2]
                    gname = 'Ghost'+str(self.camera_number)+str(Cam.id)
                    if (gname,str(ids[id][0])) in self.robot.world.world_map.temp_ghosts:
                        self.robot.world.world_map.temp_ghosts[gname,str(ids[id][0])].update(X, Y, 0, phi, self.uncertainity(self.corners))
                        self.seen[gname,str(ids[id][0])]=True
                    else:
                        print(gname+'-'+str(ids[id][0]))
                        self.robot.world.world_map.temp_ghosts[gname,str(ids[id][0])] = RobotGhostObj(Cam.id, ids[id][0], X, Y, 0, phi, True, self.uncertainity(self.corners), Cam.calibration_number)
                        self.seen[gname,str(ids[id][0])]=True
                except:
                    print('Camera not found')
            self.generate_walls_from_markers( self.corners, ids)
        for key in self.seen:
            self.robot.world.world_map.temp_ghosts[key].is_visible = self.seen[key]

        self.post_completion()

class FindTransforms(StateNode):
    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        for key, value in self.robot.world.world_map.temp_ghosts.items():
            if value.calibration_number != self.supreme_cozmo_id and value.calibration_number == value.cozmo_id and (value.calibration_number,self.supreme_cozmo_id) not in self.parent.tranforms:
                for k, v in self.robot.world.world_map.temp_ghosts.items():
                    if v.calibration_number == self.supreme_cozmo_id and v.cozmo_id == value.cozmo_id:
                        self.supreme = v
                        break

                theta = value.theta - self.supreme.theta

                X = self.supreme.x -( cos(theta)*value.x + sin(theta)*value.y )
                Y = self.supreme.y -( cos(theta)*value.y - sin(theta)*value.x )
                self.parent.tranforms[(value.calibration_number,self.supreme_cozmo_id)] = (theta,X,Y)

                print("Added Transform from",value.calibration_number,"to",self.supreme_cozmo_id)

        self.post_completion()

class Fusion(StateNode):
    def transform_ghost(self, ghost):
        theta, X, Y = self.parent.tranforms[ghost.calibration_number,self.supreme_cozmo_id]
        x = X + cos(theta)*ghost.x + sin(theta)*ghost.y
        y = Y + cos(theta)*ghost.y - sin(theta)*ghost.x
        ghost.x=x
        ghost.y=y
        ghost.theta-=theta
        ghost.calibration_number = self.supreme_cozmo_id

    def transform_cam(self, cam):
        try:
            theta, X, Y = self.parent.tranforms[cam.calibration_number,self.supreme_cozmo_id]
            x = X + cos(theta)*cam.x + sin(theta)*cam.y
            y = Y + cos(theta)*cam.y - sin(theta)*cam.x
            cam.x=x
            cam.y=y
            cam.phi+=theta
            cam.calibration_number = self.supreme_cozmo_id
        except:
            print("No transform found")

    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        minn = {}
        for key, value in self.robot.world.world_map.temp_ghosts.items():
            if value.is_visible and int(key[1]) in self.parent.cozmo_map:
                cm = minn.get(key[1],np.inf)
                if cm > value.uncertainity:
                    minn[key[1]] = value.uncertainity
                    if value.calibration_number != self.supreme_cozmo_id:
                        self.transform_ghost(value)
                if self.parent.cozmo_map[int(key[1])] != socket.gethostname():
                    self.robot.world.world_map.objects['Ghost'+key[1]]=value
                else:
                    self.robot.world.particle_filter.set_pose(value.x, value.y, value.theta)
        for key, value in self.robot.world.world_map.temp_cams.items():
            if 'Cam' in key and value.calibration_number != self.supreme_cozmo_id:
                self.transform_cam(value)

            self.robot.world.world_map.objects[key] = value

        self.post_completion()

class Client(object):
    def __init__(self, robot):
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
        return self #lets user to call client = Client().startClient()

    def sendMessage(self,msg):
        self.socket.recv(1024)
        if type(msg) == str:
            self.socket.sendall((msg).encode()) #send as byte string
        else:
            self.socket.sendall(pickle.dumps(msg))


class myThread (threading.Thread):
    def __init__(self, threadID, name, client, robot):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.c = client
        self.robot = robot
    def run(self):
        for i in range(300):
            self.c.send(b'Thank you for connecting')
            ghosts = pickle.loads(self.c.recv(4096))

            for key, value in ghosts.items():
                if 'Cam' in key:
                    self.robot.world.world_map.temp_cams[key]=value
                else:
                    self.robot.world.world_map.temp_ghosts[key]=value


class Server(threading.Thread):
    def __init__(self, robot, port=42):
        threading.Thread.__init__(self)
        self.port = port
        self.socket = None #not running until startServer is called
        self.robot= robot

    def run(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setblocking(True) #lets select work properly I think
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True) #enables easy server restart
        self.socket.bind(("",self.port)) #binds to any address the computer can be accessed by

        self.threadLock = threading.Lock()
        self.threads =[]

        for i in range(100):
            self.socket.listen(5)   # Now wait for client connection.
            c, addr = self.socket.accept()    # Establish connection with client.
            print('Got connection from', addr)
            self.threads.append(myThread(i, "Client"+str(i), c, self.robot))
            self.threads[i].start()

class Send(StateNode):
    def start(self,event=None):
        super().start(event)
        ghosts = dict(dict())
        for key, value in self.robot.world.world_map.objects.items():
            if type(key)==str:
                ghosts[key] = value

        for key, value in self.robot.world.world_map.temp_ghosts.items():
            ghosts[key] = value

        self.parent.client.sendMessage(ghosts)
        self.post_completion()


class Sharedmap(StateMachineProgram):
    def start(self):
        super().__init__(cam_viewer=False)
        print("Enter your server's ip address:",end='')
        ipaddr = '' #input().strip() # get user input to be server ip
        if ipaddr not in ["None",""]:
            print("ipaddr is "+ipaddr)
            self.client = Client(self.robot).startClient(ipaddr=ipaddr,port=1800)
        else:
            print("Launching Server...")
            self.server = Server(self.robot, port=1800)
            self.server.start()
        self.cozmo_map = { 1:"a", 2:"tekkotsu2" }
        self.tranforms = {}
        self.caplist = {}
        self.supreme_cozmo_id = 1

        self.cameraMatrix = np.matrix([[1345.060313, 0.000000, 739.328158],
        [0.000000, 1357.301748, 396.351370],
        [0.000000, 0.000000, 1.000000]])
        self.distCoeffs = np.array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])
        super().start()

    def setup(self):
        """
            #launch:  Recieve() =C=> process
            launch:  LocateCam(0) =C=> process
            #setup :  ProcessImage(1,1140)  =C=> FindTransforms() =C=> Set_current_cozmo() =C=> process
            #process: Recieve() =C=> process
            process: ProcessImage(0) =C=> FindTransforms() =C=> Fusion() =C=> process
        """
        
        # Code generated by genfsm on Thu Oct 12 14:10:43 2017:
        
        launch = LocateCam(0) .set_name("launch") .set_parent(self)
        process = ProcessImage(0) .set_name("process") .set_parent(self)
        findtransforms1 = FindTransforms() .set_name("findtransforms1") .set_parent(self)
        fusion1 = Fusion() .set_name("fusion1") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(launch) .add_destinations(process)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(process) .add_destinations(findtransforms1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(findtransforms1) .add_destinations(fusion1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(fusion1) .add_destinations(process)
        
        return self


