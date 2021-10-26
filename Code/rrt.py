__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np


from pathplanning import PathPlanningProblem, Rectangle, Obstacle
from map_navigation import A3_map


MAP_WIDTH = 100
MAP_HEIGHT = 100

NUM_OBSTACLE = 10
MAX_OBSTACLE_WIDTH = 50
MAX_OBSTACLE_HEIGHT = 50
MIN_OBSTACLE_WIDTH = 10
MIN_OBSTACLE_HEIGHT = 10

OBSTACLE_OVERLAPPED = True

MAX_RRT_STEP_LENGTH = 5



class RRT:

    class Node:
        #pos = (x, y)
        #child = [(x1, y1), (x1, y1), .... , (xn, yn)]
        #parent = (x, y)
        def __init__(self, x, y, parent=None, child=[]):
            self.x = x
            self.y = y
            self.child = child
            self.parent = parent

        def dist(self, node):
            return math.sqrt((self.x-node.x)**2 + (self.y-node.y)**2)
        def angle(self, node):
            return math.atan2(node.y- self.y, node.x-self.x)


    def __init__(self, map, start, goal, max_step_length=3,biased_rate=0.05, max_step=50000, ANIMATION=False, SHOW_RESULT = True ):
        self.map = map
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.biased_rate = biased_rate
        self.max_step = max_step
        self.max_step_length = max_step_length
        self.tree = [self.Node(start[0], start[1])]
        self.ANIMATION=ANIMATION
        self.SHOW_RESULT = SHOW_RESULT


    def planningPath(self, ax=None):
        path = []
        for i in range(self.max_step):
            newNode = self.getRandomNode()
            closestNode = self.findClosestNode(newNode)

            if newNode.dist(closestNode) < self.max_step_length:
                nextStep = newNode
            else:
                theta = closestNode.angle(newNode)
                nextStep = self.Node(closestNode.x + self.max_step_length*math.cos(theta), closestNode.y + self.max_step_length*math.sin(theta))

            isValidNode = True
            for obstacle in self.map.obstacles:
                if RRT.isRectAndLineIntersection((closestNode.x, closestNode.y, nextStep.x, nextStep.y), obstacle):
                    isValidNode = False
                    break

            if not isValidNode:
                continue
            nextStep.parent = closestNode
            closestNode.child.append(nextStep)
            self.tree.append(nextStep)

            if i%1==0:
                if self.ANIMATION and ax is not None:
                    self.displayMap(ax,newNode,nextStep)
                    self.displayMap(ax, newNode, nextStep, path=path)
                    plt.draw()
                    plt.show()
                    plt.pause(0.001)

            if nextStep.dist(self.goal) < self.max_step_length:
                self.goal.parent = nextStep
                nextStep.child.append(self.goal)
                self.tree.append(self.goal)

                prevNode = self.goal
                while prevNode is not None:
                    path.append(prevNode)
                    prevNode = prevNode.parent
                if self.SHOW_RESULT and ax is not None:
                    self.displayMap(ax,newNode, nextStep, path=path)
                    plt.draw()
                    plt.show()
                    plt.pause(0.001)
                break
        return path

    def displayMap(self,ax,newNode=None,nextStep=None, path=[]):


        ax.clear()
        ax.set_xlim(0.0, self.map.width)
        ax.set_ylim(0.0, self.map.width)

        for o in self.map.obstacles:
            ax.add_patch(o.patch)

        ip = plt.Rectangle((self.start.x-1.5, self.start.y-1.5), 3, 3, facecolor='#00ff00')
        ax.add_patch(ip)

        g = plt.Rectangle((self.goal.x-1.5, self.goal.y-1.5), 3, 3, facecolor='#ff0000')
        ax.add_patch(g)
        if newNode is not None:
            ax.plot(newNode.x, newNode.y, 'mo',markersize=3)

        if nextStep is not None:
            ax.plot(nextStep.x, nextStep.y, 'c^', markersize=3)


        ax.set_title('Vacuuming Domain')
        for theNode in self.tree:
            if theNode.parent is not None:
                ax.plot([theNode.x, theNode.parent.x], [theNode.y,theNode.parent.y],'b-', linewidth=1)

        for theNode in path:
            if theNode.parent is not None:
                ax.plot([theNode.x, theNode.parent.x], [theNode.y, theNode.parent.y], 'r-', linewidth=2)




    def close_display_window(self):

        plt.close(self.fig)




    def findClosestNode(self, node):
        closestNode = None
        closestDist = self.map.width + self.map.height
        for theNode in self.tree:
            dist = node.dist(theNode)
            if dist < closestDist:
                closestNode = theNode
                closestDist = dist
        return closestNode


    def getRandomNode(self):
        if(random.uniform(0,1) > self.biased_rate):
            return RRT.Node(random.uniform(0,self.map.width), random.uniform(0, self.map.height))
        else:
            return self.goal

    #line= (x1,y1,x2,y2)
    #rect= (x,y,width,height)
    @staticmethod
    def isRectAndLineIntersection(line, rect):
        line_P1 = (line[0], line[1])
        line_p2 = (line[2], line[3])
        rect_p1 = (rect.x, rect.y)
        rect_p2 = (rect.x+rect.width, rect.y)
        rect_p3 = (rect.x+rect.width, rect.y+rect.height)
        rect_p4 = (rect.x, rect.y + rect.height)

        return RRT.isLineIntersection(line_P1, line_p2, rect_p1, rect_p2) \
               or RRT.isLineIntersection(line_P1, line_p2, rect_p2, rect_p3) \
               or RRT.isLineIntersection(line_P1, line_p2, rect_p3, rect_p4) \
               or RRT.isLineIntersection(line_P1, line_p2, rect_p4, rect_p1)

    @staticmethod
    def isLineIntersection(l1_P1, l1_P2, l2_P1, l2_P2):
        x1 = l1_P1[0]
        y1 = l1_P1[1]
        x2 = l1_P2[0]
        y2 = l1_P2[1]
        x3 = l2_P1[0]
        y3 = l2_P1[1]
        x4 = l2_P2[0]
        y4 = l2_P2[1]
        if ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)) == 0:
            uA = sys.maxsize
            uB =sys.maxsize
        else:
            uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
            uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
            return True
        return False




def ExploreDomain( domain, initial, steps ):
    log = np.zeros((steps,2))
    pos = np.array(initial)
    dd = 0.1
    theta = 0.00/180.0 * math.pi

    for i in range(steps):
        newpos = pos + dd * np.array([dd * math.cos(theta), dd * math.sin(theta)])
        r = Rectangle(newpos[0], newpos[1], 0.1, 0.1)
        if ( newpos[0] >= 0.0 ) and ( newpos[0] < domain.width ) and ( newpos[1] >= 0.0 ) and ( newpos[1] < domain.height ):
            if ( not domain.CheckOverlap( r ) ):
                pos = newpos

        theta = theta + random.uniform(-180.0/180.0 * math.pi, 180.0/180.0 * math.pi)
        while( theta >= math.pi ):
            theta = theta - 2 * math.pi
        while( theta < - math.pi ):
            theta = theta + 2 * math.pi
        log[i,:] = pos
    return log

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    fig = plt.figure()
    plt.ion()
    ax = fig.add_subplot(1, 1, 1, aspect='equal')

    pp = A3_map( MAP_WIDTH, MAP_HEIGHT,  NUM_OBSTACLE, MAX_OBSTACLE_WIDTH, MAX_OBSTACLE_HEIGHT,  MIN_OBSTACLE_WIDTH, MIN_OBSTACLE_HEIGHT, overlapped=True)
    initial, goals = pp.CreateProblemInstance()
    for g in goals:
        rrt = RRT(pp, initial, g,ANIMATION = True)
        rrt.planningPath(ax)




if ( __name__ == '__main__' ):
    main()
    input('press enter key to exit:')

