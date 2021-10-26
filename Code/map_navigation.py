from pathplanning import PathPlanningProblem, Obstacle, Rectangle
import random
import matplotlib.pyplot as plt
import numpy as np

class A3_map(PathPlanningProblem):
    def __init__(self, width, height, onum, oMaxWidth, oMaxHeight,  oMinWidth=0, oMinHeight=0, overlapped=False):
        self.width = width
        self.height = height
        self.obstacles = self.CreateObstacles(onum, oMaxWidth, oMaxHeight,oMinWidth,oMinHeight,overlapped)

    def CreateObstacles(self, onum, oMaxWidth, oMaxHeight, oMinWidth=0, oMinHeight=0, overlapped=False):
        obstacles = []

        while( len(obstacles) < onum ):
            x = random.uniform(0.0, self.width)
            y = random.uniform(0.0, self.height)
            w = round(random.uniform(oMinWidth, oMaxWidth))
            h = round(random.uniform(oMinHeight, oMaxHeight))
            if ( x + w ) > self.width:
                w = self.width - x
            if ( y + h ) > self.height:
                h = self.height - y
            obs = Obstacle(x,y, w, h, '#808080')
            found = False
            if overlapped:
                obstacles = obstacles + [obs]
            else:
                for o in obstacles:
                    if ( o.CalculateOverlap(obs) > 0.0 ):
                        found = True
                        break
                if ( not found ):
                    obstacles = obstacles + [obs]
        return obstacles