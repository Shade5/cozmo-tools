from cozmo.util import Pose
from cv2 import Rodrigues
from numpy import matrix
from pdb import set_trace



from .nodes import *
from .transitions import *
from .transform import wrap_angle
from .pilot import PilotToPose, PilotCheckStart

from math import sin, cos, atan2, pi, sqrt

class GoToWall(StateNode):

    def __init__(self, wall=None, door_id = 0):
        self.object = wall
        self.wall_name = 'Wall-'+str(wall)
        self.door_id = door_id
        super().__init__()
        if self.wall_name in self.robot.world.world_map.objects:
            self.wobj = self.robot.world.world_map.objects[self.wall_name]
            print("Choosing Robot view")
        else:
            self.wobj = self.robot.world.world_map.objects["GhostWall"+str(wall)]
            print("Choosing Ghost")

        self.door_coordinates = self.wobj.markers[door_id][1]

        print(self.door_coordinates)

    def pick_side(self, dist):
        wall = self.object
        door_coordinates = self.door_coordinates
        x = self.wobj.x 
        y = self.wobj.y
        ang = self.wobj.theta
        rx = self.robot.world.particle_filter.pose[0]
        ry = self.robot.world.particle_filter.pose[1]

        side1 = (x + cos(ang)*dist - sin(ang)*(self.wobj.length/2 - door_coordinates[0]), y + sin(ang)*dist + cos(ang)*( self.wobj.length/2 - door_coordinates[0]), ang + pi)
        side2 = (x - cos(ang)*dist - sin(ang)*(self.wobj.length/2 - door_coordinates[0]), y - sin(ang)*dist + cos(ang)*( self.wobj.length/2 - door_coordinates[0]), ang)
        sides = [side1, side2]
        sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)

        return sorted_sides[0]

    class TurnToSide(Turn):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            wall = self.parent.object
            wobj = self.parent.wobj
            dtheta = (wobj.theta - self.robot.world.particle_filter.pose_estimate()[2])
            if abs(dtheta) > 0.1:
                self.angle = Angle(dtheta)
                super().start(event)
            else:
                self.angle = Angle(0)
                super().start(event)
                self.post_success()


    class GoToSide(PilotToPose):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            wall = self.parent.object
            print('Selected wall',self.parent.wobj)
            (x, y, theta) = self.parent.pick_side(200)

            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians = wrap_angle(theta)))
            print('Traveling to',self.target_pose)
            super().start(event)


    class ReportPosition(StateNode):
        def start(self,event=None):
            super().start(event)
            wall = self.parent.object
            wobj = self.parent.wobj
            cx = wobj.x
            cy = wobj.y
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            dx = cx - rx
            dy = cy - ry
            dist = math.sqrt(dx*dx + dy*dy)
            bearing = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) * 180/pi
            print('wall at (%5.1f,%5.1f)  robot at (%5.1f,%5.1f)  dist=%5.1f  brg=%5.1f' %
                  (cx, cy, rx, ry, dist, bearing))


    class TurnToWall(Turn):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            door_id = self.parent.door_id
            if door_id not in self.robot.world.aruco.seen_marker_ids:
                print('** TurnToWall could not see the Door.')
                self.angle = Angle(degrees=0)
                super().start(event)
                self.post_failure()
            else:
                while True:
                    rx = self.robot.pose.position.x
                    ry = self.robot.pose.position.y
                    rt = self.robot.pose.rotation.angle_z.radians

                    marker = self.robot.world.aruco.seen_marker_objects.get(door_id,0)
                    if marker!=0:
                        break

                sensor_dist = marker.camera_distance
                sensor_bearing = atan2(marker.camera_coords[0],
                                       marker.camera_coords[2])
                sensor_orient = - marker.opencv_rotation[1] * (pi/180)

                direction = rt + sensor_bearing
                dx = sensor_dist * cos(direction)
                dy = sensor_dist * sin(direction)
                cx = rx + dx
                cy = ry + dy
                dist = math.sqrt(dx*dx + dy*dy)
                self.angle = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) \
                             * 180/pi
                if abs(self.angle) < 2:
                    self.angle = 0
                self.angle = Angle(degrees=self.angle)
                print("TurnToWall", self.angle)
                super().start(event)


    class ForwardToWall(Forward):
        def __init__(self, offset):
            self.offset = offset
            super().__init__()

        def start(self, event=None):
            if self.running: return
            door_id = self.parent.door_id
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            rt = self.robot.pose.rotation.angle_z.radians

            marker = self.robot.world.aruco.seen_marker_objects[door_id]
            sensor_dist = marker.camera_distance
            sensor_bearing = atan2(marker.camera_coords[0],
                                   marker.camera_coords[2])
            sensor_orient = - marker.opencv_rotation[1] * (pi/180)

            direction = rt + sensor_bearing
            dx = sensor_dist * cos(direction)
            dy = sensor_dist * sin(direction)
            cx = rx + dx
            cy = ry + dy
            dist = math.sqrt(dx*dx + dy*dy)
            self.distance = Distance(sqrt(dx*dx + dy*dy) - self.offset)
            super().start(event)


    class FindWall(SetHeadAngle):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            if self.running: return
            door_id = self.parent.door_id
            if door_id not in self.robot.world.aruco.seen_marker_ids:
                print('Looking higher for wall')
                if self.robot.head_angle.degrees < 30:
                    self.angle = Angle(self.robot.head_angle.radians + 0.4)
                    super().start(event)
                else:
                    self.angle = self.robot.head_angle
                    super().start(event)
            else:
                self.angle = self.robot.head_angle
                super().start(event)


    def setup(self):
        """
            droplift: SetLiftHeight(0) =T(0.5)=> check_start    # time for vision to set up world map
    
            check_start: PilotCheckStart()
            check_start =S=> SetHeadAngle(0) =T(2)=> turn_to_side
            check_start =F=> Forward(-80) =C=> check_start
    
            turn_to_side: self.TurnToSide()
            turn_to_side =C=> turn_to_side
            turn_to_side =S=> self.ReportPosition() =N=> go_side
    
            go_side: self.GoToSide() =C=> self.TurnToSide() =C=> lookup
    
            lookup:  SetHeadAngle(25) =T(2)=> find
    
            find: self.TurnToWall() =C=>
                self.ReportPosition() =T(0.75)=> approach
            find =F=> Forward(-80) =C=> StateNode() =T(1)=> find2
    
            find2: self.TurnToWall() =C=>
                self.ReportPosition() =T(0.75)=> approach
            find2 =F=> Forward(-80) =C=> Say("No Door trying again") =C=> turn_to_side
    
            approach: self.ForwardToWall(150) =C=> self.FindWall() =C=>
                self.ReportPosition() =T(0.75)=>
                self.TurnToWall() =C=> self.FindWall() =C=>
                self.ForwardToWall(100) =C=> self.FindWall() =C=>
                self.ReportPosition() =T(0.75)=>
                self.TurnToWall() =C=> self.FindWall() =C=>
                self.ForwardToWall(60) =C=> self.FindWall() =C=>
                self.ReportPosition() =T(0.75)=>
                self.TurnToWall()=C=> end
    
    
            end: SetHeadAngle(0) =C=> Forward(100) =C=> Say("Done") =C=> ParentCompletes()
        """
        
        # Code generated by genfsm on Tue Oct 31 12:06:49 2017:
        
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        setheadangle1 = SetHeadAngle(0) .set_name("setheadangle1") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        turn_to_side = self.TurnToSide() .set_name("turn_to_side") .set_parent(self)
        reportposition1 = self.ReportPosition() .set_name("reportposition1") .set_parent(self)
        go_side = self.GoToSide() .set_name("go_side") .set_parent(self)
        turntoside1 = self.TurnToSide() .set_name("turntoside1") .set_parent(self)
        lookup = SetHeadAngle(25) .set_name("lookup") .set_parent(self)
        find = self.TurnToWall() .set_name("find") .set_parent(self)
        reportposition2 = self.ReportPosition() .set_name("reportposition2") .set_parent(self)
        forward2 = Forward(-80) .set_name("forward2") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        find2 = self.TurnToWall() .set_name("find2") .set_parent(self)
        reportposition3 = self.ReportPosition() .set_name("reportposition3") .set_parent(self)
        forward3 = Forward(-80) .set_name("forward3") .set_parent(self)
        say1 = Say("No Door trying again") .set_name("say1") .set_parent(self)
        approach = self.ForwardToWall(150) .set_name("approach") .set_parent(self)
        findwall1 = self.FindWall() .set_name("findwall1") .set_parent(self)
        reportposition4 = self.ReportPosition() .set_name("reportposition4") .set_parent(self)
        turntowall1 = self.TurnToWall() .set_name("turntowall1") .set_parent(self)
        findwall2 = self.FindWall() .set_name("findwall2") .set_parent(self)
        forwardtowall1 = self.ForwardToWall(100) .set_name("forwardtowall1") .set_parent(self)
        findwall3 = self.FindWall() .set_name("findwall3") .set_parent(self)
        reportposition5 = self.ReportPosition() .set_name("reportposition5") .set_parent(self)
        turntowall2 = self.TurnToWall() .set_name("turntowall2") .set_parent(self)
        findwall4 = self.FindWall() .set_name("findwall4") .set_parent(self)
        forwardtowall2 = self.ForwardToWall(60) .set_name("forwardtowall2") .set_parent(self)
        findwall5 = self.FindWall() .set_name("findwall5") .set_parent(self)
        reportposition6 = self.ReportPosition() .set_name("reportposition6") .set_parent(self)
        turntowall3 = self.TurnToWall() .set_name("turntowall3") .set_parent(self)
        end = SetHeadAngle(0) .set_name("end") .set_parent(self)
        forward4 = Forward(100) .set_name("forward4") .set_parent(self)
        say2 = Say("Done") .set_name("say2") .set_parent(self)
        parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
        
        timertrans1 = TimerTrans(0.5) .set_name("timertrans1")
        timertrans1 .add_sources(droplift) .add_destinations(check_start)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_start) .add_destinations(setheadangle1)
        
        timertrans2 = TimerTrans(2) .set_name("timertrans2")
        timertrans2 .add_sources(setheadangle1) .add_destinations(turn_to_side)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(forward1) .add_destinations(check_start)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(turn_to_side) .add_destinations(turn_to_side)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(turn_to_side) .add_destinations(reportposition1)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(reportposition1) .add_destinations(go_side)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(go_side) .add_destinations(turntoside1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(turntoside1) .add_destinations(lookup)
        
        timertrans3 = TimerTrans(2) .set_name("timertrans3")
        timertrans3 .add_sources(lookup) .add_destinations(find)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(find) .add_destinations(reportposition2)
        
        timertrans4 = TimerTrans(0.75) .set_name("timertrans4")
        timertrans4 .add_sources(reportposition2) .add_destinations(approach)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(find) .add_destinations(forward2)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(forward2) .add_destinations(statenode1)
        
        timertrans5 = TimerTrans(1) .set_name("timertrans5")
        timertrans5 .add_sources(statenode1) .add_destinations(find2)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(find2) .add_destinations(reportposition3)
        
        timertrans6 = TimerTrans(0.75) .set_name("timertrans6")
        timertrans6 .add_sources(reportposition3) .add_destinations(approach)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(find2) .add_destinations(forward3)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(forward3) .add_destinations(say1)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(say1) .add_destinations(turn_to_side)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(approach) .add_destinations(findwall1)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(findwall1) .add_destinations(reportposition4)
        
        timertrans7 = TimerTrans(0.75) .set_name("timertrans7")
        timertrans7 .add_sources(reportposition4) .add_destinations(turntowall1)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(turntowall1) .add_destinations(findwall2)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(findwall2) .add_destinations(forwardtowall1)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(forwardtowall1) .add_destinations(findwall3)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(findwall3) .add_destinations(reportposition5)
        
        timertrans8 = TimerTrans(0.75) .set_name("timertrans8")
        timertrans8 .add_sources(reportposition5) .add_destinations(turntowall2)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(turntowall2) .add_destinations(findwall4)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(findwall4) .add_destinations(forwardtowall2)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(forwardtowall2) .add_destinations(findwall5)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(findwall5) .add_destinations(reportposition6)
        
        timertrans9 = TimerTrans(0.75) .set_name("timertrans9")
        timertrans9 .add_sources(reportposition6) .add_destinations(turntowall3)
        
        completiontrans20 = CompletionTrans() .set_name("completiontrans20")
        completiontrans20 .add_sources(turntowall3) .add_destinations(end)
        
        completiontrans21 = CompletionTrans() .set_name("completiontrans21")
        completiontrans21 .add_sources(end) .add_destinations(forward4)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(forward4) .add_destinations(say2)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(say2) .add_destinations(parentcompletes1)
        
        return self
