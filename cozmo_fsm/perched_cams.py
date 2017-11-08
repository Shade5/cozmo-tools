import cv2
import cv2.aruco as aruco
import numpy as np
import threading
from pdb import set_trace
from time import sleep

class Cam():
    def __init__(self,cap,x,y,z):
        self.cap = cap
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '<Cam (%.2f, %.2f, %.2f)>' % \
               (self.x, self.y, self.z,)

class Perched_cams(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.use_perched_cameras=False
        self.cameras = []
        self.cameraMatrix = np.matrix([[1148.00,       -3,    641.0],
                                       [0.000000,   1145.0,    371.0],
                                       [0.000000, 0.000000, 1.000000]])
        self.distCoeffs = np.array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        self.cams = {}

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
        self.start()

    def process_image(self):
        self.temp_cams = {}     # Necessary, else self.cams is empty most of the time
        for cap in self.cameras:
            for i in range(5):
                cap.grab()
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if type(ids) is np.ndarray: # and self.comzo_aruco in ids:
                vecs = aruco.estimatePoseSingleMarkers(corners, 50, self.cameraMatrix, self.distCoeffs)
                rvecs, tvecs = vecs[0], vecs[1]
                for i in range(len(ids)):
                    rotationm, jcob = cv2.Rodrigues(rvecs[i])
                    transformed = np.matrix(rotationm).T*(-np.matrix(tvecs[i]).T)
                    if ids[i][0] in self.temp_cams:
                        self.temp_cams[ids[i][0]][cap]=Cam(cap,transformed[0][0,0],transformed[1][0,0],transformed[2][0,0])
                    else:
                        self.temp_cams[ids[i][0]]={cap:Cam(cap,transformed[0][0,0],transformed[1][0,0],transformed[2][0,0])}
        self.cams = self.temp_cams



