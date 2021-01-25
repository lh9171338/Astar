import numpy as np
import cv2
import heapq


class Node:
    point = None        # node coordinate
    F = None            # cost
    G = None
    H = None
    parent = None       # parent node

    def __init__(self, point=(0, 0)):
        self.point = tuple(point)  
        self.F = 0.0
        self.G = 0.0
        self.H = 0.0

class AstarConfig:
    euclidean = None
    occupyThresh = None
    inflateRadius = None

    def __init__(self, euclidean=True, occupyThresh=-1, inflateRadius=-1):
        self.euclidean = euclidean
        self.occupyThresh = occupyThresh
        self.inflateRadius = inflateRadius


class Astar:
    # Map
    __occMap = None
    __infMap = None
    __labelMap = None
    __ParentMap = None

    __startPoint = None
    __targetPoint = None
    __neighbors = None
    __config = None

    __openList = None    # open list
    __openDict = None
    __pathList = None    # path list

    # Node type
    OBSTACLE = 0
    FREE = 1
    INOPENLIST = 2
    INCLOSELIST = 3

    def __init__(self):
        self.__neighbors = [[-1, -1], [-1, 0], [-1, 1],
            [0, -1], [0, 1],
            [1, -1], [1, 0], [1, 1]]

    def initAstar(self, occMap, config=AstarConfig()):
        self.__occMap = occMap
        self.__config = config
        self.__inflateMap()

        return self.__infMap

    def pathPlanning(self, startPoint, targetPoint):
        self.__startPoint = tuple(startPoint)
        self.__targetPoint = tuple(targetPoint)

        tailNode = self.__findPath()
        path = self.__getPath(tailNode)
        return path

    def drawPath(self):
        pass

    def __inflateMap(self):
        height, width = self.__occMap.shape[0], self.__occMap.shape[1]
        occMap = self.__occMap.copy()

        # Transform RGB to gray image
        if len(occMap.shape) == 3:
            occMap = cv2.cvtColor(occMap.copy(), cv2.COLOR_BGR2GRAY)

        # Binarize
        if self.__config.occupyThresh < 0:
            cv2.threshold(occMap.copy(), 0, 255, cv2.THRESH_OTSU, occMap)
        else:
            cv2.threshold(occMap.copy(), self.__config.occupyThresh, 255, cv2.THRESH_BINARY, occMap)
        
        # Inflate
        src = occMap.copy()
        if self.__config.inflateRadius > 0:
            se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.__config.inflateRadius, 2 * self.__config.inflateRadius))
            cv2.erode(src.copy(), se, occMap)

        # Get mask
        self.__infMap = cv2.bitwise_xor(src, occMap)

        # Initial LabelMap
        labelMap = np.zeros((height, width), np.uint8)
        labelMap[occMap == 0] = self.OBSTACLE
        labelMap[occMap > 0] = self.FREE
        self.__labelMap = labelMap

    def __findPath(self):
        height, width = self.__occMap.shape[0], self.__occMap.shape[1]
        labelMap = self.__labelMap.copy()

        # Add startPoint to OpenList
        startPointNode = Node(self.__startPoint)
        self.__openList = []
        self.__openDict = dict()
        heapq.heappush(self.__openList, (startPointNode.F, startPointNode.point))  
        self.__openDict[startPointNode.point] = startPointNode
        labelMap[self.__startPoint[1], self.__startPoint[0]] = self.INOPENLIST

        while len(self.__openList):
            # Find the node with least F value
            curPoint = heapq.heappop(self.__openList)[1]
            if curPoint not in self.__openDict:
                continue
            curNode = self.__openDict[curPoint]
            del self.__openDict[curPoint]

            curX, curY = curNode.point
            labelMap[curY, curX] = self.INCLOSELIST

            # Determine whether arrive the target point
            if curNode.point == self.__targetPoint:
                return curNode  # Find a valid path
            
            # Traversal the neighborhood
            for neighbor in self.__neighbors:
                x = curX + neighbor[0]
                y = curY + neighbor[1]
                if x < 0 or x >= width or y < 0 or y >= height:
                    continue
                if labelMap[y, x] in [self.FREE, self.INOPENLIST]:
                    # Determine whether a diagonal line can pass
                    dist = abs(neighbor[0]) + abs(neighbor[1])
                    if dist == 2 and labelMap[y, curX] == self.OBSTACLE and labelMap[curY, x] == self.OBSTACLE:
                        continue
                    
                    # Calculate G, H, F value
                    addG = 14 if dist == 2 else 10
                    G = curNode.G + addG
                    if self.__config.euclidean:
                        dist = np.sqrt((x - self.__targetPoint[0]) ** 2 + (y - self.__targetPoint[1]) ** 2)
                        H = round(10 * dist)
                    else:
                        H = 10 * (abs(x - self.__targetPoint[0]) + abs(y - self.__targetPoint[1]))
                    F = G + H

                    # Update the G, H, F value of node
                    if labelMap[y, x] == self.FREE or (labelMap[y, x] == self.INOPENLIST and G < self.__openDict[(x, y)].G):
                        newNode = Node()
                        newNode.point = (x, y)
                        newNode.G = G
                        newNode.H = H
                        newNode.F = F
                        newNode.parent = curNode
                        heapq.heappush(self.__openList, (newNode.F, newNode.point))  
                        self.__openDict[newNode.point] = newNode
                        labelMap[y, x] = self.INOPENLIST 
                
        return None

    def __getPath(self, tailNode):
        path = []

        curNode = tailNode
        while curNode != None:
           path.append(curNode.point) 
           curNode = curNode.parent

        return path[::-1]


