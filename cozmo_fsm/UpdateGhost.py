import cv2
from cozmo_fsm import *

class LocateCam2(StateNode):
    """ Locates Camera1."""
    def __init__(self, camera_number=1):
        self.camera_number = camera_number # Set according to camera
        super().__init__()

    def getframe(self):
        for i in range(5):
            self.cap.grab()
        ret, self.frame = self.cap.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        return self.gray

    def getcorners(self):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def distance_to_aruco(self, corners):
        return 5*np.sqrt(self.focus**2 + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )/np.linalg.norm(corners[0][:][0][0] - corners[0][:][0][1])


    def start(self, event=None):
        super().start(event)

        self.focus = 1140     # Set according to camera
        self.camera_width = 1280
        self.camera_height = 720
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,self.camera_width)
        self.cap.set(4,self.camera_height)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()

        while(True):
            self.getframe()
            self.corners, self.ids1 = self.getcorners()
            if type(self.ids1) is np.ndarray:
                (x0, y0) = self.corners[0][0][0]
                (x1, y1) = self.corners[0][0][1]
                (x2, y2) = self.corners[0][0][2]
                (x3, y3) = self.corners[0][0][3]

                omega_x = np.arctan((360 - np.mean(self.corners[0][0][:,1]))/self.focus)
                omega_y = np.arctan((640 - np.mean(self.corners[0][0][:,0]))/self.focus)

                A = (x0-x3)#/np.cos(omega_x)
                C = (x1-x0)#/np.cos(omega_x)
                phi = np.arctan2(A,C)
               
                comparr =  phi*180/np.pi
                
                if -30 < comparr < 30 or 150 < comparr < 180 or -180 < comparr < -150:
                    theta = np.mean( (np.arccos((y3-y0)/(x1-x0)), np.arccos((y2-y1)/(x2-x3))) ) - omega_y   
                elif 30 <= comparr < 60 or -150 < comparr <= -120:
                    theta = np.arccos((y2-y0)/(x1-x3)) - omega_y  
                elif -60 <= comparr <= -30 or 120 <= comparr <= 150:
                    theta = np.arccos((y3-y1)/(x2-x0)) - omega_y
                else:
                    theta = np.mean( (np.arccos((y2-y3)/(x0-x3)), np.arccos((y1-y0)/(x1-x2))) ) - omega_y
                                      
                l = np.sqrt(A**2 + C**2)                
                r = np.sqrt(self.focus*self.focus + (np.mean(self.corners[0][0][:,0])-640)**2 + (np.mean(self.corners[0][0][:,1])-360)**2 )
                R1 = 50*r/l
                
                Y = (np.mean(self.corners[0][0][:,0])-640)*R1/r
                X = (360 - np.mean(self.corners[0][0][:,1]))*R1/(r*np.cos( theta + omega_x ))
                
                height = np.sqrt( R1*R1 - X*X )*np.cos(theta+omega_x)             
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        print("Released")

        self.theta = theta
        self.height = height + 100
        self.phi = self.robot.pose.rotation.angle_z.radians + phi
        
        camera_x = self.robot.pose.position._x - (self.height*np.tan(self.theta + omega_x)*np.cos(self.phi) + Y*np.sin(self.phi))
        camera_y = self.robot.pose.position._y - ( -self.height*np.tan(self.theta + omega_x)*np.sin(self.phi) + Y*np.cos(self.phi))

        print("Theta:",self.theta*180/np.pi,"Phi",self.phi*180/np.pi, "Camera at: ",camera_x, camera_y)
        
        self.robot.world.world_map.objects['Cam1'] = CameraObj(1, camera_x, -camera_y, self.height, self.theta, self.phi, (X,Y) )
        self.robot.world.world_map.objects['Ghost1'] = RobotGhostObj(1, self.robot.pose.position._x, self.robot.pose.position._y, self.robot.pose.position._z, self.robot.pose.rotation.angle_z.radians)
     
        self.post_completion()



class ProcessImage(StateNode):
    def __init__(self):
        super().__init__()
        self.flag=1

    def __del__(self):
        super().__del__()
        self.cap.release()

    def start(self, event=None):
        if self.flag:
	        self.cap = cv2.VideoCapture(1)
	        self.cap.set(3,1280)
	        self.cap.set(4,720)
	        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	        self.parameters =  aruco.DetectorParameters_create()
	        self.F = 1140
	        self.flag=0
        super().start(event)

        #Update Ghost
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if type(ids) is np.ndarray:
            (x0, y0) = corners[0][0][0]
            (x1, y1) = corners[0][0][1]
            (x2, y2) = corners[0][0][2]
            (x3, y3) = corners[0][0][3]
      
            A = (x0-x3)#/np.cos(omega_x)
            C = (x1-x0)#/np.cos(omega_x)
        
            gphi = np.arctan2(A,C)
            phi = self.robot.world.world_map.objects['Cam1'].phi
            angle = self.robot.world.world_map.objects['Cam1'].theta
            initial_position = self.robot.world.world_map.objects['Cam1'].initial_position
        
            l = np.sqrt(A**2 + C**2)
            r = np.sqrt(self.F*self.F + (np.mean(corners[0][0][:,0])-640)**2 + (np.mean(corners[0][0][:,1])-360)**2 )
            R1 = 50*r/l

            X = (360 - np.mean(corners[0][0][:,1]))*R1/(r*np.cos(angle)) #- initial_position[0]
            Y = -( ((np.mean(corners[0][0][:,0])-640)*R1/r)) #- initial_position[1] )

            self.robot.world.world_map.objects['Ghost1'].theta =  -gphi + phi
            self.robot.world.world_map.objects['Ghost1'].x = X*cos(phi) - Y*sin(phi) 
            self.robot.world.world_map.objects['Ghost1'].y = X*sin(phi) + Y*cos(phi) 

class UpdateGhost(StateMachineProgram):
    def start(self):
        super().start()
    def setup(self):
        """
        	launch:  LocateCam2(1) =C=> process
            process: ProcessImage() =T(0.050)=> process
        """
        
        # Code generated by genfsm on Wed Aug 16 15:14:22 2017:
        
        launch = LocateCam2(1) .set_name("launch") .set_parent(self)
        process = ProcessImage() .set_name("process") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(launch) .add_destinations(process)
        
        timertrans1 = TimerTrans(0.050) .set_name("timertrans1")
        timertrans1 .add_sources(process) .add_destinations(process)
        
        return self
