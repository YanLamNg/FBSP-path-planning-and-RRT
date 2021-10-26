__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
from Code.map_navigation import A3_map

from Code.pathplanning import PathPlanningProblem, Rectangle




class CellDecomposition:

    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', [], -1, None]

    def Draw(self, ax, node=None):
        if (node == None):
            node = self.root
        r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
        if (node[1] == 'mixed'):
            color = '#5080ff'
            if (node[2] == []):
                r.set_fill(True)
                r.set_facecolor(color)
        elif (node[1] == 'free'):
            color = '#ffff00'
            r.set_fill(True)
            r.set_facecolor(color)
        elif (node[1] == 'obstacle'):
            color = '#5050ff'
            r.set_fill(True)
            r.set_facecolor(color)
        else:
            print("Error: don't know how to draw cell of type", node[1])
        # print('Draw node', node)
        ax.add_patch(r)
        for c in node[2]:
            self.Draw(ax, c)

    def CountCells(self, node=None):
        if (node is None):
            node = self.root
        if (node[2] != []):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum


# noinspection PyUnresolvedReferences
class QuadTreeDecomposition(CellDecomposition):

    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.map_nodes = []
        self.mixed_nodes = []
        self.root = self.Decompose(self.root)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        for o in self.domain.obstacles:
            if (o.CalculateOverlap(r) >= rwidth * rheight):
                cell = 'obstacle'
                break
            elif (o.CalculateOverlap(r) > 0.0):
                cell = 'mixed'
                break
        if (cell == 'mixed'):
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                childt1 = [Rectangle(rx, ry, rwidth / 2.0, rheight / 2.0), 'unknown', [], -1, None]
                qchild1 = self.Decompose(childt1)
                childt2 = [Rectangle(rx + rwidth / 2.0, ry, rwidth / 2.0, rheight / 2.0), 'unknown', [], -1, None]
                qchild2 = self.Decompose(childt2)
                childt3 = [Rectangle(rx, ry + rheight / 2.0, rwidth / 2.0, rheight / 2.0), 'unknown', [], -1, None]
                qchild3 = self.Decompose(childt3)
                childt4 = [Rectangle(rx + rwidth / 2.0, ry + rheight / 2.0, rwidth / 2.0, rheight / 2.0), 'unknown', [],
                           -1, None]
                qchild4 = self.Decompose(childt4)
                children = [qchild1, qchild2, qchild3, qchild4]
                node[2] = children
                self.mixed_nodes.append(node)
            else:
                cell = 'obstacle'
        elif cell == 'free':
            self.map_nodes.append(node)

        node[1] = cell
        return node

    def Contains(self, target, nodes):
        for node in nodes:
            r = node[0]
            if target[0] >= r.x and target[0] <= r.x + r.width and target[1] >= r.y and target[1] <= r.y + r.height:
                return node

        return None

    def Center(self, node):
        r = node[0]
        return np.array([r.x + r.width / 2, r.y + r.height / 2])

    def Distance(self, start, end):
        return math.sqrt(
            (self.Center(start)[0] - self.Center(end)[0]) ** 2 + (self.Center(start)[1] - self.Center(end)[1]) ** 2)

    def FindMinDistance(self, nodes, goal):
        if nodes is not None:
            min = nodes[0]
            for n in nodes:
                if n[3] + self.Distance(n, goal) < min[3] + self.Distance(min, goal):
                    min = n
        return min

    def GetNeighbors(self, curr):
        neighbors = []
        for node in self.map_nodes:
            if (node[0].x == curr[0].x + curr[0].width) or (curr[0].x == node[0].x + node[0].width):
                if node[0].y <= curr[0].y and (curr[0].y + curr[0].height <= node[0].y + node[0].height):
                    neighbors.append(node)
                elif curr[0].y <= node[0].y and (node[0].y + node[0].height <= curr[0].y + curr[0].height):
                    neighbors.append(node)
            elif (node[0].y == curr[0].y + curr[0].height) or (curr[0].y == node[0].y + node[0].height):
                if node[0].x <= curr[0].x and (curr[0].x + curr[0].width <= node[0].x + node[0].width):
                    neighbors.append(node)
                elif curr[0].x <= node[0].x and (node[0].x + node[0].width <= curr[0].x + curr[0].width):
                    neighbors.append(node)
        return neighbors

    def AStar(self, start, goal):
        openSet = []
        closeSet = []
        # G score and H score

        if self.Contains(goal, self.map_nodes) is None:
            goal = self.Contains(goal, self.mixed_nodes)
        else:
            goal = self.Contains(goal, self.map_nodes)
        if self.Contains(start, self.map_nodes) is None:
            openSet.append(self.Contains(start, self.mixed_nodes))
        else:
            openSet.append(self.Contains(start, self.map_nodes))

        openSet[0][3] = 0
        while len(openSet) > 0:
            current = self.FindMinDistance(openSet, goal)
            # if we reach the target, return cost
            if self.Distance(current, goal) == 0:
                path = []
                length = current[3]
                while current[4]:
                    path.append(current)
                    current = current[4]
                path.append(current)
                return length, path

            openSet.remove(current)
            closeSet.append(current)

            for node in self.GetNeighbors(current):
                if node in openSet:
                    temp_g = current[3] + self.Distance(current, node)
                    if node[3] is not -1 and node[3] > temp_g:
                        node[3] = temp_g
                        node[4] = current
                elif node in closeSet:
                    continue
                else:
                    node[3] = current[3] + self.Distance(current, node)
                    node[4] = current
                    openSet.append(node)
        return -1, None


class FlexibleBinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.map_nodes = []
        self.mixed_nodes = []
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ((p > 0) and (p < 1.0)):
            e = -p * math.log(p, 2) - (1 - p) * math.log(1 - p, 2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        for o in self.domain.obstacles:
            a = a + rect.CalculateOverlap(o)
        p = a / area
        return self.Entropy(p)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight
        partitions_h = []
        partitions_v = []
        for o in self.domain.obstacles:
            if (o.CalculateOverlap(r) >= rwidth * rheight):
                cell = 'obstacle'
                break
            elif (o.CalculateOverlap(r) > 0.1):
                partitions_h.append([o.x, o.y])
                partitions_h.append([o.x, o.y + o.height])
                partitions_v.append([o.x, o.y])
                partitions_v.append([o.x + o.width, o.y])
                cell = 'mixed'
                break

        if (cell == 'mixed'):
            entropy = self.CalcEntropy(r)

            igH, hSplitTop, hSplitBottom = self.CalMinGain(partitions_h, entropy, rx, ry, rwidth, rheight, 'horizontal')
            igV, vSplitLeft, vSplitRight = self.CalMinGain(partitions_v, entropy, rx, ry, rwidth, rheight, 'vertical')

            children = []
            if (igH > igV):
                if (igH > 0.0):
                    if (hSplitTop is not None) and (hSplitBottom is not None):
                        childTop = [hSplitTop, 'unknown', [], -1, None]
                        childBottom = [hSplitBottom, 'unknown', [], -1, None]
                        children = [childTop, childBottom]
            else:
                if (igV > 0.0):
                    if (vSplitLeft is not None) and (vSplitRight is not None):
                        childLeft = [vSplitLeft, 'unknown', [], -1, None]
                        childRight = [vSplitRight, 'unknown', [], -1, None]
                        children = [childLeft, childRight]
            for c in children:
                self.Decompose(c)
            self.mixed_nodes.append(node)
            node[2] = children
        elif cell == 'free':
            self.map_nodes.append(node)
        node[1] = cell
        return node

    def CalMinGain(self, partitions, entropy, rx, ry, rwidth, rheight, type):
        maxIg = 0
        max_splitPartA = None
        max_splitPartB = None
        area = rwidth * rheight
        if type == 'horizontal':
            for point in partitions:
                if ry + rheight - point[1] > self.minimumSize and point[1] - ry > self.minimumSize:
                    splitPartA = Rectangle(rx, point[1], rwidth, ry + rheight - point[1])
                    ent1 = self.CalcEntropy(splitPartA)
                    splitPartB = Rectangle(rx, ry, rwidth, point[1] - ry)
                    ent2 = self.CalcEntropy(splitPartB)
                    ig = entropy - (rwidth * (ry + rheight - point[1])) / area * ent1 \
                         - (rwidth * (point[1] - ry)) / area * ent2
                    if ig > maxIg:
                        maxIg = ig
                        max_splitPartA = splitPartA
                        max_splitPartB = splitPartB
        elif type == 'vertical':
            for point in partitions:
                if point[0] - rx > self.minimumSize and rx + rwidth - point[0] > self.minimumSize:
                    splitPartA = Rectangle(rx, ry, point[0] - rx, rheight)
                    ent1 = self.CalcEntropy(splitPartA)
                    splitPartB = Rectangle(point[0], ry, rx + rwidth - point[0], rheight)
                    ent2 = self.CalcEntropy(splitPartB)
                    ig = entropy - ((point[0] - rx) * rheight) / area * ent1 \
                         - ((rx + rwidth - point[0]) * rheight) / area * ent2
                    if ig > maxIg:
                        maxIg = ig
                        max_splitPartA = splitPartA
                        max_splitPartB = splitPartB

        return maxIg, max_splitPartA, max_splitPartB

    def Contains(self, target, nodes):
        for node in nodes:
            r = node[0]
            if target[0] >= r.x and target[0] <= r.x + r.width and target[1] >= r.y and target[1] <= r.y + r.height:
                return node

        return None

    def Center(self, node):
        r = node[0]
        return np.array([r.x + r.width / 2, r.y + r.height / 2])

    def Distance(self, start, end):
        return math.sqrt(
            (self.Center(start)[0] - self.Center(end)[0]) ** 2 + (self.Center(start)[1] - self.Center(end)[1]) ** 2)

    def FindMinDistance(self, nodes, goal):
        if nodes is not None:
            min = nodes[0]
            for n in nodes:
                if n[3] + self.Distance(n, goal) < min[3] + self.Distance(min, goal):
                    min = n
        return min

    def GetNeighbors(self, curr):
        neighbors = []
        for node in self.map_nodes:
            if (node[0].x == curr[0].x + curr[0].width) or (curr[0].x == node[0].x + node[0].width):
                if node[0].y <= curr[0].y and (curr[0].y + curr[0].height <= node[0].y + node[0].height):
                    neighbors.append(node)
                elif curr[0].y <= node[0].y and (node[0].y + node[0].height <= curr[0].y + curr[0].height):
                    neighbors.append(node)
            elif (node[0].y == curr[0].y + curr[0].height) or (curr[0].y == node[0].y + node[0].height):
                if node[0].x <= curr[0].x and (curr[0].x + curr[0].width <= node[0].x + node[0].width):
                    neighbors.append(node)
                elif curr[0].x <= node[0].x and (node[0].x + node[0].width <= curr[0].x + curr[0].width):
                    neighbors.append(node)
        return neighbors

    def AStar(self, start, goal):
        openSet = []
        closeSet = []
        # G score and H score
        if self.Contains(goal, self.map_nodes) is None:
            goal = self.Contains(goal, self.mixed_nodes)
        else:
            goal = self.Contains(goal, self.map_nodes)
        if self.Contains(start, self.map_nodes) is None:
            openSet.append(self.Contains(start, self.mixed_nodes))
        else:
            openSet.append(self.Contains(start, self.map_nodes))

        openSet[0][3] = 0
        while len(openSet) > 0:
            current = self.FindMinDistance(openSet, goal)
            # if we reach the target, return cost
            if self.Distance(current, goal) == 0:
                path = []
                length = current[3]
                while current[4]:
                    path.append(current)
                    current = current[4]
                path.append(current)
                return length, path

            openSet.remove(current)
            closeSet.append(current)

            for node in self.GetNeighbors(current):
                if node in openSet:
                    temp_g = current[3] + self.Distance(current, node)
                    if node[3] is not -1 and node[3] > temp_g:
                        node[3] = temp_g
                        node[4] = current
                elif node in closeSet:
                    continue
                else:
                    node[3] = current[3] + self.Distance(current, node)
                    node[4] = current
                    openSet.append(node)
        return -1, None


def main(argv=None):
    if (argv == None):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0
    minWidth = 10.0
    minHeight = 10.0
    maxWidth = 50.0
    maxHeight = 50.0

    numObstacles = 10
    if len(argv) > 0:
        numObstacles = int(argv[0])

    pp = A3_map(width, height, numObstacles, maxWidth, maxHeight, minWidth, minHeight, overlapped=False)
    # pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))
    ip = plt.Rectangle((initial[0], initial[1]), 1, 1, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0], g[1]), 1, 1, facecolor='#00ff00')
        ax.add_patch(g)

    qtd = QuadTreeDecomposition(pp, 0.2)
    length, path = qtd.AStar(initial, goals[0])

    if path:
        for i in range(len(path)):
            node = path[i]
            if node[4] is not None:
                if i == 0:
                    ax.plot([goals[0][0], qtd.Center(node)[0]], [goals[0][1], qtd.Center(node)[1]], 'b-')
                    ax.plot([qtd.Center(node)[0], qtd.Center(node[4])[0]],
                            [qtd.Center(node)[1], qtd.Center(node[4])[1]], 'b-')
                    length += math.sqrt((goals[0][0]-qtd.Center(node)[0])**2+(goals[0][1]-qtd.Center(node)[1])**2)
                else:
                    ax.plot([qtd.Center(node)[0], qtd.Center(node[4])[0]],
                            [qtd.Center(node)[1], qtd.Center(node[4])[1]], 'b-')
            if i == len(path)-1:
                ax.plot([qtd.Center(node)[0], initial[0]], [qtd.Center(node)[1], initial[1]], 'b-')
                length += math.sqrt((qtd.Center(node)[0] - initial[0]) ** 2 + (qtd.Center(node)[1] - initial[1]) ** 2)

        print("Quadtree path length: {0}".format(length))
    else:
        print("Quadtree path not found")

    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    ax = fig.add_subplot(1,2,2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))
    ip = plt.Rectangle((initial[0],initial[1]), 0.1, 0.1, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 0.1, 0.1, facecolor='#00ff00')
        ax.add_patch(g)

    fbsp = FlexibleBinarySpacePartitioning(pp, 0.05)
    length, path = fbsp.AStar(initial, goals[0])

    if path:
        for i in range(len(path)):
            node = path[i]
            if node[4] is not None:
                if i == 0:
                    ax.plot([goals[0][0], fbsp.Center(node)[0]], [goals[0][1], fbsp.Center(node)[1]], 'b-')
                    ax.plot([fbsp.Center(node)[0], fbsp.Center(node[4])[0]],
                            [fbsp.Center(node)[1], fbsp.Center(node[4])[1]], 'b-')
                    length += math.sqrt((goals[0][0]-fbsp.Center(node)[0])**2+(goals[0][1]-fbsp.Center(node)[1])**2)
                else:
                    ax.plot([fbsp.Center(node)[0], fbsp.Center(node[4])[0]],
                            [fbsp.Center(node)[1], fbsp.Center(node[4])[1]], 'b-')
            if i == len(path) - 1:
                ax.plot([fbsp.Center(node)[0], initial[0]], [fbsp.Center(node)[1], initial[1]], 'b-')
                length += math.sqrt((fbsp.Center(node)[0] - initial[0]) ** 2 + (fbsp.Center(node)[1] - initial[1]) ** 2)
        print("FBSP path length: {0}".format(length))
    else:
        print("FBSP path not found")

    fbsp.Draw(ax)
    n = fbsp.CountCells()
    ax.set_title('FBSP Decomposition\n{0} cells'.format(n))

    plt.show()


if (__name__ == '__main__'):
    main()
