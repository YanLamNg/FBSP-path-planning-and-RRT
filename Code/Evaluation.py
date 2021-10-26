
import math
import matplotlib.pyplot as plt
import os,sys

import os,sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
FBSP  = __import__("fbsp path planning")
import time
import rrt
import map_navigation as mn
import copy
MAP_WIDTH = 100
MAP_HEIGHT = 100



OBSTACLE_OVERLAPPED = True

MAX_RRT_STEP_LENGTH = 2
NUM_TEST_TYPE = 4
NUM_OBSTACLE = [10,20,30,40]
MAX_OBSTACLE_WIDTH = [50,35,25,20]
MAX_OBSTACLE_HEIGHT = [50,35,25,20]
MIN_OBSTACLE_WIDTH = [10,9,8,7]
MIN_OBSTACLE_HEIGHT = [10,9,8,7]
NUM_TEST_COUNT = 5

display_path = False
save_fig = True

def runRRT(pp,initial, goals, fig):
    if fig is not None:
        ax = fig.add_subplot(1, 3, 3, aspect='equal')
        ax.set_xlim(0.0, MAP_WIDTH)
        ax.set_ylim(0.0, MAP_HEIGHT)
        ax.set_title('RRT')
    else:
        ax = None

    path_length = 0
    runtime = 0
    if goals != []:
        g = goals[0]
        start_time = time.time()
        rrt_problem = rrt.RRT(pp, initial, g,max_step_length=MAX_RRT_STEP_LENGTH, max_step=50000, SHOW_RESULT = False)
        paths = rrt_problem.planningPath()
        runtime = time.time() - start_time
        for path in paths:
            if path.parent is not None:
                path_length += path.dist(path.parent)
        if paths == []:
            print("RRT path not found")
        if fig is not None:
            rrt_problem.displayMap(ax, path=paths)

    return path_length, runtime

def run_fbsp(pp,initial, goals, fig=None):
    if fig is not None:
        ax = fig.add_subplot(1, 3, 1, aspect='equal')
        ax.set_xlim(0.0, MAP_WIDTH)
        ax.set_ylim(0.0, MAP_HEIGHT)

        for o in pp.obstacles:
            ax.add_patch(copy.copy(o.patch))
        ip = plt.Rectangle((initial[0], initial[1]), 1, 1, facecolor='#ff0000')
        ax.add_patch(ip)

        for g in goals:
            g = plt.Rectangle((g[0], g[1]), 1, 1, facecolor='#00ff00')
            ax.add_patch(g)

    start_time = time.time()
    qtd = FBSP.QuadTreeDecomposition(pp, 0.2)
    length, path = qtd.AStar(initial, goals[0])

    if path:
        for i in range(len(path)):
            node = path[i]
            if node[4] is not None:
                if i == 0:
                    if fig is not None:
                        ax.plot([goals[0][0], qtd.Center(node)[0]], [goals[0][1], qtd.Center(node)[1]], 'b-')
                        ax.plot([qtd.Center(node)[0], qtd.Center(node[4])[0]],
                                [qtd.Center(node)[1], qtd.Center(node[4])[1]], 'b-')
                    length += math.sqrt((goals[0][0]-qtd.Center(node)[0])**2+(goals[0][1]-qtd.Center(node)[1])**2)
                elif  fig is not None:
                    ax.plot([qtd.Center(node)[0], qtd.Center(node[4])[0]],
                            [qtd.Center(node)[1], qtd.Center(node[4])[1]], 'b-')
            if i == len(path)-1:
                if fig is not None: ax.plot([qtd.Center(node)[0], initial[0]], [qtd.Center(node)[1], initial[1]], 'b-')
                length += math.sqrt((qtd.Center(node)[0] - initial[0]) ** 2 + (qtd.Center(node)[1] - initial[1]) ** 2)

    #     print("Quadtree path length: {0}".format(length))
    else:
        print("Quadtree path not found")
    quadTree_length = length
    quadTree_runtime = time.time() - start_time
    if fig is not None:
        qtd.Draw(ax)
        n = qtd.CountCells()
        ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

        ax = fig.add_subplot(1,3,2, aspect='equal')
        ax.set_xlim(0.0, MAP_WIDTH)
        ax.set_ylim(0.0, MAP_HEIGHT)

        for o in pp.obstacles:
            ax.add_patch(copy.copy(o.patch))
        ip = plt.Rectangle((initial[0],initial[1]), 0.1, 0.1, facecolor='#ff0000')
        ax.add_patch(ip)

        for g in goals:
            g = plt.Rectangle((g[0],g[1]), 0.1, 0.1, facecolor='#00ff00')
            ax.add_patch(g)

    start_time = time.time()
    fbsp = FBSP.FlexibleBinarySpacePartitioning(pp, 0.05)
    length, path = fbsp.AStar(initial, goals[0])
    FBSP_runtime = time.time() - start_time
    if path:
        for i in range(len(path)):
            node = path[i]
            if node[4] is not None:
                if i == 0:
                    if fig is not None:
                        ax.plot([goals[0][0], fbsp.Center(node)[0]], [goals[0][1], fbsp.Center(node)[1]], 'b-')
                        ax.plot([fbsp.Center(node)[0], fbsp.Center(node[4])[0]],
                                [fbsp.Center(node)[1], fbsp.Center(node[4])[1]], 'b-')
                    length += math.sqrt((goals[0][0]-fbsp.Center(node)[0])**2+(goals[0][1]-fbsp.Center(node)[1])**2)
                elif fig is not None:
                    ax.plot([fbsp.Center(node)[0], fbsp.Center(node[4])[0]],
                            [fbsp.Center(node)[1], fbsp.Center(node[4])[1]], 'b-')
            if i == len(path) - 1:
                if fig is not None:ax.plot([fbsp.Center(node)[0], initial[0]], [fbsp.Center(node)[1], initial[1]], 'b-')
                length += math.sqrt((fbsp.Center(node)[0] - initial[0]) ** 2 + (fbsp.Center(node)[1] - initial[1]) ** 2)
    #     print("FBSP path length: {0}".format(length))
    else:
         print("FBSP path not found")
    FBSP_length = length
    if fig is not None:
        fbsp.Draw(ax)
        n = fbsp.CountCells()
        ax.set_title('FBSP Decomposition\n{0} cells'.format(n))


    return quadTree_runtime, quadTree_length, FBSP_runtime,FBSP_length



if ( __name__ == '__main__' ):
    plt.ioff()
    if display_path or save_fig:
        fig = plt.figure(figsize=(12,5))
        counter = 0
    else:
        fig = None

    for i in range(NUM_TEST_TYPE):
        total_RRT_path_length = 0.0
        total_RRT_runtime = 0.0
        total_quadTree_runtime = 0.0
        total_quadTree_length = 0.0
        total_FBSP_runtime = 0.0
        total_FBSP_length = 0.0

        for j in range(NUM_TEST_COUNT):
            if display_path or save_fig:fig.clf()
            runtime = 0.0
            path_length = 5
            pp = mn.A3_map( MAP_WIDTH, MAP_HEIGHT,  NUM_OBSTACLE[i], MAX_OBSTACLE_WIDTH[i], MAX_OBSTACLE_HEIGHT[i],  MIN_OBSTACLE_WIDTH[i], MIN_OBSTACLE_HEIGHT[i], overlapped=False)
            initial, goals = pp.CreateProblemInstance()
            quadTree_runtime, quadTree_length, FBSP_runtime, FBSP_length = run_fbsp(pp, initial, goals,fig)
            RRT_path_length, RRT_runtime = runRRT(pp, initial, goals,fig)

            total_RRT_runtime += RRT_runtime
            total_RRT_path_length += RRT_path_length
            total_quadTree_runtime += quadTree_runtime
            total_quadTree_length += quadTree_length
            total_FBSP_runtime += FBSP_runtime
            total_FBSP_length += FBSP_length
            if display_path:
                plt.show()

            if save_fig:
                plt.savefig('../figure/fig{}_with_{}_obstacle.png'.format(counter,NUM_OBSTACLE[i]), dpi=150)
                counter += 1


        print('/*********************  Map info  ***************************')
        print('Map_size: {0} \nnumber_obstacle: {1}\nobstacle_max_size: {2}\nobstacle_min_size: {3} \nnum_sameple: {4}'.format( MAP_WIDTH, NUM_OBSTACLE[i], MAX_OBSTACLE_WIDTH[i], MIN_OBSTACLE_WIDTH[i], NUM_TEST_COUNT))
        print('************************************************************/')
        print('\tRRT average path length: {}'.format(total_RRT_path_length/NUM_TEST_COUNT))
        print('\tRRT average runtime: {}'.format(total_RRT_runtime/NUM_TEST_COUNT))
        print('\tQuadTree average path length: {}'.format(total_quadTree_length / NUM_TEST_COUNT))
        print('\tQuadTree average runtime: {}'.format(total_quadTree_runtime / NUM_TEST_COUNT))
        print('\tFBSP average path length: {}'.format(total_FBSP_length / NUM_TEST_COUNT))
        print('\tFBSP average runtime: {}\n'.format(total_FBSP_runtime / NUM_TEST_COUNT))

