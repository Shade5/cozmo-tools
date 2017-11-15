import cv2
import cv2.aruco as aruco
from numpy import matrix, array, ndarray, sqrt, arctan2, pi
import threading
from pdb import set_trace
from time import sleep
from .transform import wrap_angle

class Cam():
    def __init__(self,cap,x,y,z,phi, theta):
        self.cap = cap
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta

    def __repr__(self):
        return '<Cam (%.2f, %.2f, %.2f)> @ %.2f' % \
               (self.x, self.y, self.z,self.phi*180/pi)

class Perched_cams(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
        self.use_perched_cameras=False
        self.cameras = []
        # Special case make live_cam matrix
        self.cameraMatrix = matrix([[1148.00,       -3,    641.0],
                                       [0.000000,   1145.0,    371.0],
                                       [0.000000, 0.000000, 1.000000]])
        self.distCoeffs = array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        self.cams = {}
        self.pool = {}

    def run(self):
        while(True):
            self.process_image()
            sleep(0.01)

    def start_pcams(self,cams):
        self.use_perched_cameras=True
        self.cameras = [cv2.VideoCapture(x) for x in cams]
        for cap in self.cameras:
            cap.set(3,4000)
            cap.set(4,4000)
        self.robot.world.particle_filter.sensor_model.use_perched_cameras = True
        print("Particle filter now using perched cameras")
        self.start()

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
     
        return array([x, y, z])

    def process_image(self):
        # Dict with key: aruco id with values as cameras that can see the marker
        self.temp_cams = {}     # Necessary, else self.cams is empty most of the time
        for cap in self.cameras:
            # Clearing Buffer by grabbing five frames
            for i in range(5):
                cap.grab()
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if type(ids) is ndarray: # and self.comzo_aruco in ids:

                vecs = aruco.estimatePoseSingleMarkers(corners, 50, self.cameraMatrix, self.distCoeffs)
                rvecs, tvecs = vecs[0], vecs[1]
                for i in range(len(ids)):
                    rotationm, jcob = cv2.Rodrigues(rvecs[i])
                    transformed = matrix(rotationm).T*(-matrix(tvecs[i]).T)
                    phi = self.rotationMatrixToEulerAngles(rotationm.T)
                    if ids[i][0] in self.temp_cams:
                        self.temp_cams[ids[i][0]][str(cap)]=Cam(str(cap),transformed[0][0,0],transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))
                    else:
                        self.temp_cams[ids[i][0]]={str(cap):Cam(str(cap),transformed[0][0,0],transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))}
        self.cams = self.temp_cams
        if self.robot.world.is_server:
            self.pool = self.temp_cams



