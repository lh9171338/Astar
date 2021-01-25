import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from occupancygridparam import OccupancyGridParam
from astar import Astar, AstarConfig


class GlobalPlanner:
    occGridParam = None
    astar = None
    inflateMap = None
    rate = None

    startPoint = None
    targetPoint = None

    def __init__(self):
        self.occGridParam = OccupancyGridParam()
        self.astar = Astar()
        self.inflateMap = OccupancyGrid()

        # Flag
        self.startFlag = False
        self.startPointFlag = False
        self.targetPointFlag = False

    def setup(self):
        # Parameter
        self.rate = rospy.get_param('~rate', 10) 
        euclidean = rospy.get_param('~euclidean', True) 
        occupyThresh = rospy.get_param('~occupyThresh', -1) 
        inflateRadius = rospy.get_param('~inflateRadius', 1.0) 

        # Advertise topics
        self.map_pub = rospy.Publisher('~inflate_map', OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher('~nav_path', Path, queue_size=10)

        # Subscribe topics
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.startPointCallback, queue_size=10)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.targetPointtCallback, queue_size=10)

        # Subscribe service
        occGrid = OccupancyGrid()
        rospy.wait_for_service('static_map')
        try:
            getmap = rospy.ServiceProxy('static_map', GetMap)
            occGrid = getmap().map
        except:
            rospy.logerr('Service call failed')
            return False

        inflateRadius = int(round(inflateRadius / occGrid.info.resolution))
        self.occGridParam.setOccupancyGridParam(occGrid)
        occMap = self.occGridParam.getMap()
        occMap[occMap < 0] = 100
        occMap = 100 - np.uint8(occMap)
        config = AstarConfig(euclidean, occupyThresh, inflateRadius)
        infMap = self.astar.initAstar(occMap, config)
        infMap = np.int8(np.clip(infMap, 0, 100))
        self.inflateMap = occGrid
        self.inflateMap.data = infMap.flatten()

        return True

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Publish inflate map
            self.publishInflateMap()

            if self.startFlag:
                # Start planning path
                pathList = self.astar.pathPlanning(self.startPoint, self.targetPoint)
                if len(pathList):
                    path = Path()
                    path.header.stamp = rospy.Time().now()
                    path.header.frame_id = 'map'
                    pathList = self.occGridParam.image2MapTransform(pathList)
                    for point in pathList:
                        poseStamped = PoseStamped()
                        poseStamped.header.stamp = rospy.Time().now()
                        poseStamped.header.frame_id = 'map'
                        poseStamped.pose.position.x = point[0]
                        poseStamped.pose.position.y = point[1]
                        poseStamped.pose.position.z = 0.0
                        path.poses.append(poseStamped)
                    self.path_pub.publish(path)

                    rospy.loginfo('Find a valid path successfully')
                else:
                    rospy.logerr('Can not find a valid path')
                self.startFlag = False

            # Sleep
            rate.sleep()

    def startPointCallback(self, msg):
        startPoint = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        startPoint = self.occGridParam.map2ImageTransform([startPoint])[0]
        startPoint = np.int32(np.round(startPoint))
        self.startPoint = [startPoint[0], startPoint[1]]
        self.startPointFlag = True
        self.startFlag = self.startPointFlag and self.targetPointFlag

        rospy.loginfo('startPoint: %f, %f, %d, %d', msg.pose.pose.position.x, msg.pose.pose.position.y, startPoint[0], startPoint[1])

    def targetPointtCallback(self, msg):
        targetPoint = [msg.pose.position.x, msg.pose.position.y]
        targetPoint = self.occGridParam.map2ImageTransform([targetPoint])[0]
        targetPoint = np.int32(np.round(targetPoint))
        self.targetPoint = [targetPoint[0], targetPoint[1]]
        self.targetPointFlag = True
        self.startFlag = self.startPointFlag and self.targetPointFlag        

        rospy.loginfo('targetPoint: %f, %f, %d, %d', msg.pose.position.x, msg.pose.position.y, targetPoint[0], targetPoint[1])

    def publishInflateMap(self):
        self.inflateMap.header.stamp = rospy.Time().now()
        self.map_pub.publish(self.inflateMap)
