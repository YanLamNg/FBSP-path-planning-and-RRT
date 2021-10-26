# FBSP path planning and RRT

The path planning simulation domain (2D map navigation) used in this assignment is a simple wheeled robot. Assume the dimension of the robot is a single point. The obstacles are simple rectangles that may overlap. 

Assume the size of the environment is 100cm by 100cm and the size of the obstacles is between 10cm by 10cm to 50cm by 50cm (down to 1cm accuracy). 

The first path planning algorithm is the quadtree and FBSP (flexible binary space partitioning) decomposition algorithms. Employ an A* algorithm for finding the shortest path. Assume the robot will move from one cell to the other through their centre points. 

The second path planning algorithm is the single source extend RRT (with biased) algorithm. The “step length” of your RRT algorithm should be 1cm - 5cm (depends on your domain). If the step-length is 5 cm and your RRT is within 5cm of the goal location, then you can add the last part from the tree node to the goal position.



COMP 4190
Artificial Intelligence
Assignment 3

Authors:

Xiaojian Xie 7821950

YanLam Ng 7775665

Group: 9

To run the program simply run the python code either for fbsp_path_planning.py or rrt.py. 

How to run in terminal:

1.go the the project directory "comp4190_ai_a3" 

2.run 'python Code/fbsp_path_planning.py <number of obstacles>' or

'python Code/rrt.py <number of obstacles>'

default value for number of obstacles is 10 

To run Evaluation.py and rrt.py

1.go to the Code folder

2.python Evaluation.py

OR

python rrt.py
