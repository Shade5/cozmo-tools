import cv2
from cozmo_fsm import *

class LocateCam(StateNode):
    """ Locates Camera1."""
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number # Set according to camera
        self.focus = focus
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

    def distance_to_aruco(self, corners):
        return 5*np.sqrt(self.focus**2 + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )/np.linalg.norm(corners[0][:][0][0] - corners[0][:][0][1])

    def start(self, event=None):
        super().start(event)
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,4000)
        self.cap.set(4,4000)
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
                print("Cannot find cozmo from Camera"+str(self.camera_number))
                self.cap.release()
                self.post_completion()
                return

            self.getframe()
            self.corners, self.ids1 = self.getcorners()
            if type(self.ids1) is np.ndarray:
                (x0, y0) = self.corners[0][0][0]
                (x1, y1) = self.corners[0][0][1]
                (x2, y2) = self.corners[0][0][2]
                (x3, y3) = self.corners[0][0][3]


                omega_x = np.arctan((self.camera_height/2 - np.mean(self.corners[0][0][:,1]))/self.focus)
                omega_y = np.arctan((self.camera_width/2 - np.mean(self.corners[0][0][:,0]))/self.focus)

                A = (x0-x3)#/np.cos(omega_y)
                C = (x1-x0)#/np.cos(omega_y)
                phi = np.arctan2(-A,-C)
                
                comparr =  phi*180/np.pi
                
                if -30 < comparr < 30 or 150 < comparr < 180 or -180 < comparr < -150:
                    theta = np.mean( (np.arccos((y3-y0)/(x1-x0)), np.arccos((y2-y1)/(x2-x3))) ) - omega_x    
                elif 30 <= comparr < 60 or -150 < comparr <= -120:
                    theta = np.arccos((y2-y0)/(x1-x3)) - omega_x   
                elif -60 <= comparr <= -30 or 120 <= comparr <= 150:
                    theta = np.arccos((y3-y1)/(x2-x0)) - omega_x  
                else:
                    theta = np.mean( (np.arccos((y2-y3)/(x0-x3)), np.arccos((y1-y0)/(x1-x2))) ) - omega_x
                      
                l = np.sqrt(A**2 + C**2)                
                r = np.sqrt(self.focus*self.focus + (np.mean(self.corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(self.corners[0][0][:,1])-self.camera_height/2)**2 )
                R1 = 50*r/l
                
                Y = (np.mean(self.corners[0][0][:,0])-self.camera_width/2)*R1/r
                X = (self.camera_height/2 - np.mean(self.corners[0][0][:,1]))*R1/(r*np.cos( theta ))
                
                height = np.sqrt( R1*R1 - X*X )*np.cos(theta+omega_x)             
                break


        self.cap.release()
        print("Released")

        self.theta = theta
        self.height = height + 100
        self.phi = self.robot.pose.rotation.angle_z.radians + phi
        
        camera_x = self.robot.pose.position._x - (self.height*np.tan(self.theta + omega_x)*np.cos(self.phi) + Y*np.sin(self.phi))
        camera_y = self.robot.pose.position._y - ( -self.height*np.tan(self.theta + omega_x)*np.sin(self.phi) + Y*np.cos(self.phi))

        print("Theta:",self.theta*180/np.pi,"Phi",self.phi*180/np.pi, "Camera" +str(self.camera_number)+ " at: ",camera_x, camera_y)
        
        self.robot.world.world_map.objects['Cam'+str(self.camera_number)] = CameraObj(self.camera_number, camera_x, -camera_y, self.height, self.theta, self.phi, (X,Y) )
        self.robot.world.world_map.objects['Ghost'+str(self.camera_number)] = RobotGhostObj(self.camera_number, self.robot.pose.position._x, self.robot.pose.position._y, self.robot.pose.position._z, self.robot.pose.rotation.angle_z.radians)
     
        self.post_completion()


class Findcubes(StateNode):
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number # Set according to camera
        self.focus = focus
        super().__init__()

    def fra(self):
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        return frame[:,:,2].astype(np.int16, copy=False)

    def twink(self, par, cube_location):
        coun = 0
        if par==0:
            A = cozmo.lights.red_light
            B = cozmo.lights.off_light
            C = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==1:
            B = cozmo.lights.red_light
            A = cozmo.lights.off_light
            C = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==2:
            C = cozmo.lights.red_light
            B = cozmo.lights.off_light
            A = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==3:
            D = cozmo.lights.red_light
            B = cozmo.lights.off_light
            C = cozmo.lights.off_light
            A = cozmo.lights.off_light
        else:
            print("Error")
        
        for i in range(20):
            red1 = self.fra()
            cube1.set_light_corners(A, B, C, D)
            red2 = self.fra()
            cube1.set_lights(cozmo.lights.off_light)
            
            a = abs(red1-red2)
            possib = (np.argmax(a)%1280,int(np.argmax(a)/1280))

            if abs(np.sum( np.subtract( possib , cube_location[par]) )) < 10:
                coun+=1
            else:
                cube_location[par] = possib
                coun = 0

            if coun==3:
                return cube_location

        print("Not found")


    def start(self, event=None):
        super().start(event)
        print("Finding Cubes")
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,4000)
        self.cap.set(4,4000)
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)


        cube_location =[(0,0), (0,0), (0,0), (0,0)]
            
        cube_location =  self.twink(0, cube_location)
        cube_location =  self.twink(1, cube_location)
        cube_location =  self.twink(2, cube_location)
        cube_location =  self.twink(3, cube_location)

        phi = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].phi
        angle = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].theta
        initial_position = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].initial_position

        self.robot.world.world_map.objects['CubeGhost'+str(self.camera_number)] = LightCubeGhostObj(self.camera_number, cube_location[0][0], cube_location[0][1], 0, 0, True)
        self.cap.release()
        print("Done")
        self.post_completion()


class ProcessImage(StateNode):
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number # Set according to camera
        self.focus = focus
        super().__init__()
        self.flag=1

    def __del__(self):
        super().__del__()
        self.cap.release()

    def start(self, event=None):
        if self.flag:
            self.cap = cv2.VideoCapture(self.camera_number)
            self.cap.set(3,4000)
            self.cap.set(4,4000)
            self.camera_width = self.cap.get(3)
            self.camera_height = self.cap.get(4)
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
            self.parameters =  cv2.aruco.DetectorParameters_create()
            self.focus = 1140
            self.flag=0
        super().start(event)

        #Update Ghost
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if type(ids) is np.ndarray:
            (x0, y0) = corners[0][0][0]
            (x1, y1) = corners[0][0][1]
            (x2, y2) = corners[0][0][2]
            (x3, y3) = corners[0][0][3]

            omega_x = np.arctan((self.camera_height/2 - np.mean(corners[0][0][:,1]))/self.focus)
      
            A = (x0-x3)#/np.cos(omega_x)
            C = (x1-x0)#/np.cos(omega_x)
        
            gphi = np.arctan2(-A,-C)
            phi = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].phi
            theta = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].theta
            initial_position = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].initial_position
        
            l = np.sqrt(A**2 + C**2)
            r = np.sqrt(self.focus*self.focus + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )
            R1 = 50*r/l

            X = (self.camera_height/2 - np.mean(corners[0][0][:,1]))*R1/(r*np.cos(theta)) - initial_position[0]
            Y = -( ((np.mean(corners[0][0][:,0])-self.camera_width/2)*R1/r) - initial_position[1] )

            self.robot.world.world_map.objects['Ghost'+str(self.camera_number)].theta =  -gphi + phi
            self.robot.world.world_map.objects['Ghost'+str(self.camera_number)].x = X*cos(phi) - Y*sin(phi) 
            self.robot.world.world_map.objects['Ghost'+str(self.camera_number)].y = X*sin(phi) + Y*cos(phi)
            self.robot.world.world_map.objects['Ghost'+str(self.camera_number)].is_visible = True
        else:
            self.robot.world.world_map.objects['Ghost'+str(self.camera_number)].is_visible = False

class UpdateGhost(StateMachineProgram):
    def start(self):
        super().start()
    def setup(self):
        """
        	launch:  LocateCam(2,750) =C=> cam2
            cam2:    LocateCam(1,1140) =C=> process
            process: ProcessImage(2,750) =T(0.050)=> ProcessImage(1,1140) =T(0.050)=>process
        """
        
        # Code generated by genfsm on Wed Sep  6 14:40:59 2017:
        
        launch = LocateCam(2,750) .set_name("launch") .set_parent(self)
        cam2 = LocateCam(1,1140) .set_name("cam2") .set_parent(self)
        process = ProcessImage(2,750) .set_name("process") .set_parent(self)
        processimage1 = ProcessImage(1,1140) .set_name("processimage1") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(launch) .add_destinations(cam2)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(cam2) .add_destinations(process)
        
        timertrans1 = TimerTrans(0.050) .set_name("timertrans1")
        timertrans1 .add_sources(process) .add_destinations(processimage1)
        
        timertrans2 = TimerTrans(0.050) .set_name("timertrans2")
        timertrans2 .add_sources(processimage1) .add_destinations(process)
        
        return self
