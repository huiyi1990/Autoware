#!/usr/bin/env python
import rospy
from vector_map_msgs.msg import *
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray

import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from twisted.python.hook import addPost





class VectorMapConvert():
    def __init__(self):
        self.pub_rviz = rospy.Publisher('/local_vector_map',MarkerArray,latch=True,queue_size=1)
        self.pub_area = rospy.Publisher('/local_vector_map_info/area',AreaArray,latch=True,queue_size=1)
        self.pub_box =rospy.Publisher('/local_vector_map_info/box',BoxArray,latch=True,queue_size=1)
        self.pub_cross_road =rospy.Publisher('/local_vector_map_info/cross_road',CrossRoadArray,latch=True,queue_size=1)
        self.pub_cross_walk =rospy.Publisher('/local_vector_map_info/cross_walk',CrossWalkArray,latch=True,queue_size=1)
        self.pub_curb =rospy.Publisher('/local_vector_map_info/curb',CurbArray,latch=True,queue_size=1)
        self.pub_curve_mirror =rospy.Publisher('/local_vector_map_info/curve_mirror',CurveMirrorArray,latch=True,queue_size=1)
        self.pub_drive_on_portion =rospy.Publisher('/local_vector_map_info/drive_on_portion',DriveOnPortionArray,latch=True,queue_size=1)
        self.pub_dtlane =rospy.Publisher('/local_vector_map_info/dtlane',DTLaneArray,latch=True,queue_size=1)
        self.pub_fence =rospy.Publisher('/local_vector_map_info/fence',FenceArray,latch=True,queue_size=1)
        self.pub_guard_rail =rospy.Publisher('/local_vector_map_info/guard_rail',GuardRailArray,latch=True,queue_size=1)
        self.pub_gutter =rospy.Publisher('/local_vector_map_info/gutter',GutterArray,latch=True,queue_size=1)
        self.pub_lane =rospy.Publisher('/local_vector_map_info/lane',LaneArray,latch=True,queue_size=1)
        self.pub_line =rospy.Publisher('/local_vector_map_info/line',LineArray,latch=True,queue_size=1)
        self.pub_node =rospy.Publisher('/local_vector_map_info/node',NodeArray,latch=True,queue_size=1)
        self.pub_point =rospy.Publisher('/local_vector_map_info/point',PointArray,latch=True,queue_size=1)
        self.pub_pole =rospy.Publisher('/local_vector_map_info/pole',PoleArray,latch=True,queue_size=1)
        self.pub_rail_crossing =rospy.Publisher('/local_vector_map_info/rail_crossing',RailCrossingArray,latch=True,queue_size=1)
        self.pub_road_edge =rospy.Publisher('/local_vector_map_info/road_edge',RoadEdgeArray,latch=True,queue_size=1)
        self.pub_road_mark =rospy.Publisher('/local_vector_map_info/road_mark',RoadMarkArray,latch=True,queue_size=1)
        self.pub_road_pole =rospy.Publisher('/local_vector_map_info/road_pole',RoadPoleArray,latch=True,queue_size=1)
        self.pub_road_sign =rospy.Publisher('/local_vector_map_info/road_sign',RoadSignArray,latch=True,queue_size=1)
        self.pub_side_strip =rospy.Publisher('/local_vector_map_info/side_strip',SideStripArray,latch=True,queue_size=1)
        self.pub_side_walk =rospy.Publisher('/local_vector_map_info/side_walk',SideWalkArray,latch=True,queue_size=1)
        self.pub_signal =rospy.Publisher('/local_vector_map_info/signal',SignalArray,latch=True,queue_size=1)
        self.pub_stop_line =rospy.Publisher('/local_vector_map_info/stop_line',StopLineArray,latch=True,queue_size=1)
        self.pub_street_light =rospy.Publisher('/local_vector_map_info/street_light',StreetLightArray,latch=True,queue_size=1)
        self.pub_utility_pole =rospy.Publisher('/local_vector_map_info/utility_pole',UtilityPoleArray,latch=True,queue_size=1)
        self.pub_vector =rospy.Publisher('/local_vector_map_info/vector',VectorArray,latch=True,queue_size=1)
        self.pub_wall =rospy.Publisher('/local_vector_map_info/wall',WallArray,latch=True,queue_size=1)
        self.pub_way_area =rospy.Publisher('/local_vector_map_info/way_area',WayAreaArray,latch=True,queue_size=1)
        self.pub_white_line =rospy.Publisher('/local_vector_map_info/white_line',WhiteLineArray,latch=True,queue_size=1)
        self.pub_zebra_zone =rospy.Publisher('/local_vector_map_info/zebra_zone',ZebraZoneArray,latch=True,queue_size=1)
        self.pub_vmap_stat =rospy.Publisher('/local_vmap_stat',Bool,latch=True,queue_size=1)
        self.sendStatictf()
        rospy.Subscriber('/vector_map',MarkerArray,self.callbackRviz)
#         rospy.Subscriber('/vector_map_info/area',AreaArray,self.callbackArea)
#         rospy.Subscriber('/vector_map_info/box',BoxArray,self.callbackBox)
#         rospy.Subscriber('/vector_map_info/cross_road',CrossRoadArray,self.callbackCrossroad)
#         rospy.Subscriber('/vector_map_info/cross_walk',CrossWalkArray,self.callbackCrosswalk)
#         rospy.Subscriber('/vector_map_info/curb',CurbArray,self.callbackCurb)
#         rospy.Subscriber('/vector_map_info/curve_mirror',CurveMirrorArray,self.callbackMirrorarray)
#         rospy.Subscriber('/vector_map_info/drive_on_portion',DriveOnPortionArray,self.callbackDriveonportion)
        rospy.Subscriber('/vector_map_info/dtlane',DTLaneArray,self.callbackDtlane)
#         rospy.Subscriber('/vector_map_info/fence',FenceArray,self.callbackFence)
#         rospy.Subscriber('/vector_map_info/guard_rail',GuardRailArray,self.callbackGuardrail)
#         rospy.Subscriber('/vector_map_info/gutter',GutterArray,self.callbackGutter)
        rospy.Subscriber('/vector_map_info/lane',LaneArray,self.callbackLane)
        rospy.Subscriber('/vector_map_info/line',LineArray,self.callbackLine)
        rospy.Subscriber('/vector_map_info/node',NodeArray,self.callbackNode)
        rospy.Subscriber('/vector_map_info/point',PointArray,self.callbackPoint)
#         rospy.Subscriber('/vector_map_info/pole',PoleArray,self.callbackPole)
#         rospy.Subscriber('/vector_map_info/rail_crossing',RailCrossingArray,self.callbackRailcrossing)
#         rospy.Subscriber('/vector_map_info/road_edge',RoadEdgeArray,self.callbackRoadedge)
#         rospy.Subscriber('/vector_map_info/road_mark',RoadMarkArray,self.callbackRoadmark)
#         rospy.Subscriber('/vector_map_info/road_pole',RoadPoleArray,self.callbackRoadpole)
#         rospy.Subscriber('/vector_map_info/road_sign',RoadSignArray,self.callbackRoadsign)
#         rospy.Subscriber('/vector_map_info/side_strip',SideStripArray,self.callbackSidestrip)
#         rospy.Subscriber('/vector_map_info/side_walk',SideWalkArray,self.callbackSidewalk)
        rospy.Subscriber('/vector_map_info/signal',SignalArray,self.callbackSignal)
        rospy.Subscriber('/vector_map_info/stop_line',StopLineArray,self.callbackStopline)
#         rospy.Subscriber('/vector_map_info/street_light',StreetLightArray,self.callbackStreetlight)
#         rospy.Subscriber('/vector_map_info/utility_pole',UtilityPoleArray,self.callbackUtilitypole)
        rospy.Subscriber('/vector_map_info/vector',VectorArray,self.callbackVector)
#         rospy.Subscriber('/vector_map_info/wall',WallArray,self.callbackWall)
#         rospy.Subscriber('/vector_map_info/way_area',WayAreaArray,self.callbackWayarea)
#         rospy.Subscriber('/vector_map_info/white_line',WhiteLineArray,self.callbackWhiteline)
#         rospy.Subscriber('/vector_map_info/zebra_zone',ZebraZoneArray,self.callbackZebrazone)
#         rospy.Subscriber('/vmap_stat',Bool,self.callbackVmapstat)
        
    def revisePosition(self,pos):
        pos.x = pos.x + 14771
        pos.y = pos.y + 84757
        pos.z = pos.z - 39
        return pos
    
    
    def callbackRviz(self,MarkerArray):
        print (len(MarkerArray.markers))
        for i in MarkerArray.markers:
            i.header.frame_id = "local_map"
            #self.revisePosition(i.pose.position)
            for j in i.points:
                self.revisePosition(j)
        print ("in loop")   
        print (len(MarkerArray.markers))
        self.pub_rviz.publish(MarkerArray)
    
    def callbackPoint(self,PointArray):
        PointArray.header.frame_id = "local_map"
         # NOTE: Autwoare use Japan Plane Rectangular Coordinate System.
         # Therefore we swap x and y axis.
        for i in PointArray.data:
            i.bx = i.bx + 84757
            i.ly = i.ly + 14771
            i.h = i.h - 39
        self.pub_point.publish(PointArray)
    
    def callbackLine(self,LineArray):
        LineArray.header.frame_id = "local_map"
        self.pub_line.publish(LineArray)
    
    def callbackLane(self,LaneArray):
        LaneArray.header.frame_id = "local_map"
        self.pub_lane.publish(LaneArray)
    
    def callbackDtlane(self,DTLaneArray):
        DTLaneArray.header.frame_id = "local_map"
        self.pub_dtlane.publish(DTLaneArray)
    
    def callbackStopline(self,StopLineArray):
        StopLineArray.header.frame_id = "local_map"
        self.pub_stop_line.publish(StopLineArray)
    
    def callbackSignal(self,SignalArray):
        SignalArray.header.frame_id = "local_map"
        self.pub_signal.publish(SignalArray)
    
    def callbackVector(self,VectorArray):
        VectorArray.header.frame_id = "local_map"
        self.pub_vector.publish(VectorArray)
    
    def callbackNode(self,NodeArray):
        NodeArray.header.frame_id = "local_map"
        self.pub_node.publish(NodeArray)
          
    def sendStatictf(self):
        self.s_br = tf2_ros.StaticTransformBroadcaster()
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "map"
        static_tf.child_frame_id = "local_map"
        static_tf.transform.translation.x = -14771 
        static_tf.transform.translation.y = -84757
        static_tf.transform.translation.z = 39
        static_tf.transform.rotation.w = 1
        static_tf.transform.rotation.x = 0
        static_tf.transform.rotation.y = 0
        static_tf.transform.rotation.z = 0
        self.s_br.sendTransform(static_tf)
        
if __name__ == '__main__':
    rospy.init_node('vector_map_convert', anonymous=True)
    vmc = VectorMapConvert()
    rospy.spin()