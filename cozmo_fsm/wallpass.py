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

    def __init__(self, wall=None):
        self.object = wall
        super().__init__()

    def pick_side(self, dist, door=0):
        wall = self.object
        wobj = self.robot.world.world_map.objects[wall]
        x = wobj.x 
        y = wobj.y
        ang = wobj.theta
        rx = self.robot.world.particle_filter.pose[0]
        ry = self.robot.world.particle_filter.pose[1]

        side1 = (x + cos(ang) * dist - sin(ang)*( wobj.doorways[door][0] - wobj.length/2 ), y + sin(ang) * dist - cos(ang)*( wobj.doorways[door][0] - wobj.length/2 ), ang + pi)
        side2 = (x - cos(ang) * dist - sin(ang)*( wobj.doorways[door][0] - wobj.length/2 ), y - sin(ang) * dist - cos(ang)*( wobj.doorways[door][0] - wobj.length/2 ), ang)
        sides = [side1, side2]
        sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)
        return sorted_sides[0]

    def locate_marker(self, marker):
        if marker not in self.robot.world.aruco.seen_marker_ids:
            print("FAIL")
        else:
            mobj = self.robot.world.aruco.seen_marker_objects[marker]
            return mobj.opencv_translation[0]


    class GoToSide(PilotToPose):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            wall = self.parent.object
            print('Selected wall',self.robot.world.world_map.objects[wall])
            (x, y, theta) = self.parent.pick_side(150,1)
            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians = wrap_angle(theta)))
            print('Traveling to',self.target_pose)
            super().start(event)

    class ReportPosition(StateNode):
        def start(self,event=None):
            super().start(event)
            wall = self.parent.object
            wobj = self.robot.world.world_map.objects[wall]
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


    class FindDoor(SetHeadAngle):
        def __init__(self):
            self.angle =  Angle(degrees = 24)
            super().__init__()

        def start(self, event=None):
            if self.running: return
            wall = self.parent.object
            if 11 not in self.robot.world.aruco.seen_marker_ids:
                print('** Could not see the door.', self.angle)
                self.angle = Angle(degrees = self.angle.degrees + 5 )
                super().start(event)
            else:
                print('Found Door')
                super().start(event)


    class VerifyDoor(StateNode):
        def __init__(self):
            super().__init__()
        def start(self,event=None):
            super().start(event)
            set_trace()
            if 11 in self.robot.world.aruco.seen_marker_ids:
                print("YES")
                self.post_success()
            else:
                print("NO")
                self.post_failure()


    class AlightwithDoor(PilotToPose):
        def __init__(self, offset=0, check_vis=False):
            self.offset = offset
            self.check_vis = check_vis
            super().__init__()

        def start(self, event=None):
            if self.running: return
            wall = self.parent.object
            print("Aligning")
            if self.check_vis and 11 not in self.robot.world.aruco.seen_marker_ids:
                print('** Could not see the door.')
                self.angle = 0 # Angle(0)
                super().start(event)
                self.post_failure()
            else:
                
                dy = self.parent.locate_marker(11)
                self.target_pose = Pose(self.robot.pose.position.x, self.robot.pose.position.y-dy, self.robot.pose.position.z,
                                        angle_z=self.robot.pose.rotation.angle_z)
                print('Traveling to',self.target_pose)
                super().start(event)


    class ForwardToWall(Forward):
        def __init__(self, offset):
            self.offset = offset
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            dx = cube.pose.position.x - self.robot.pose.position.x
            dy = cube.pose.position.y - self.robot.pose.position.y
            self.distance = Distance(sqrt(dx*dx + dy*dy) - self.offset)
            super().start(event)

    def setup(self):
        """
            droplift: SetLiftHeight(0) =T(0.5)=> lookup    # time for vision to set up world map
    
            check_start: PilotCheckStart()
            check_start =S=> go_side
            check_start =F=> Forward(-80) =C=> check_start
    
            go_side: self.GoToSide()
            go_side =F=> ParentFails()
            go_side =C=> self.ReportPosition()
                =T(0.5)=> self.ReportPosition()
                =T(0.5)=> self.ReportPosition()
                =N=> lookup
    
            lookup:  SetHeadAngle(24) =T(2)=> find
    
            find: self.FindDoor() =C=> verify
    
            verify: self.VerifyDoor()
            verify =S=> face_wall
            verify =F=> find
    
            face_wall: self.AlightwithDoor(0,True) =C=>
                self.ReportPosition() =T(0.5)=> self.ReportPosition()
                =T(0.5)=> self.ReportPosition()=N=> end
    
            end: ParentCompletes()
        """
        
        # Code generated by genfsm on Wed Oct 11 17:13:12 2017:
        
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        go_side = self.GoToSide() .set_name("go_side") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        reportposition1 = self.ReportPosition() .set_name("reportposition1") .set_parent(self)
        reportposition2 = self.ReportPosition() .set_name("reportposition2") .set_parent(self)
        reportposition3 = self.ReportPosition() .set_name("reportposition3") .set_parent(self)
        lookup = SetHeadAngle(24) .set_name("lookup") .set_parent(self)
        find = self.FindDoor() .set_name("find") .set_parent(self)
        verify = self.VerifyDoor() .set_name("verify") .set_parent(self)
        face_wall = self.AlightwithDoor(0,True) .set_name("face_wall") .set_parent(self)
        reportposition4 = self.ReportPosition() .set_name("reportposition4") .set_parent(self)
        reportposition5 = self.ReportPosition() .set_name("reportposition5") .set_parent(self)
        reportposition6 = self.ReportPosition() .set_name("reportposition6") .set_parent(self)
        end = ParentCompletes() .set_name("end") .set_parent(self)
        
        timertrans1 = TimerTrans(0.5) .set_name("timertrans1")
        timertrans1 .add_sources(droplift) .add_destinations(lookup)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_start) .add_destinations(go_side)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(forward1) .add_destinations(check_start)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(go_side) .add_destinations(parentfails1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(go_side) .add_destinations(reportposition1)
        
        timertrans2 = TimerTrans(0.5) .set_name("timertrans2")
        timertrans2 .add_sources(reportposition1) .add_destinations(reportposition2)
        
        timertrans3 = TimerTrans(0.5) .set_name("timertrans3")
        timertrans3 .add_sources(reportposition2) .add_destinations(reportposition3)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(reportposition3) .add_destinations(lookup)
        
        timertrans4 = TimerTrans(2) .set_name("timertrans4")
        timertrans4 .add_sources(lookup) .add_destinations(find)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(find) .add_destinations(verify)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(verify) .add_destinations(face_wall)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(verify) .add_destinations(find)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(face_wall) .add_destinations(reportposition4)
        
        timertrans5 = TimerTrans(0.5) .set_name("timertrans5")
        timertrans5 .add_sources(reportposition4) .add_destinations(reportposition5)
        
        timertrans6 = TimerTrans(0.5) .set_name("timertrans6")
        timertrans6 .add_sources(reportposition5) .add_destinations(reportposition6)
        
        nulltrans2 = NullTrans() .set_name("nulltrans2")
        nulltrans2 .add_sources(reportposition6) .add_destinations(end)
        
        return self



