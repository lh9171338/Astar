import rospy
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np


class OccupancyGridParam:
    __R = None
    __t = None
    __s = None

    __map = None

    def __init__(self):
        pass

    def setOccupancyGridParam(self, occGrid):
        # Map information
        resolution = occGrid.info.resolution
        height = occGrid.info.height
        width = occGrid.info.width
        x = occGrid.info.origin.position.x
        y = occGrid.info.origin.position.y
        quat = occGrid.info.origin.orientation
        theta = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        
        # Calculate R, t 
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], np.float32)
        t = np.array([[x, y]], np.float32)
        self.__R = R
        self.__t = t
        self.__s = resolution

        # Get map
        self.__map = np.asarray(occGrid.data).reshape((height, width))

    def image2MapTransform(self, srcPoints):
        srcPoints = np.asarray(srcPoints, np.float32)
        dstPoints = self.__s * np.matmul(srcPoints, self.__R.T) + self.__t
        return dstPoints

    def map2ImageTransform(self, srcPoints):
        srcPoints = np.asarray(srcPoints, np.float32)
        dstPoints = np.matmul((srcPoints - self.__t) / self.__s, self.__R)
        return dstPoints        

    def getMap(self):
        return self.__map
