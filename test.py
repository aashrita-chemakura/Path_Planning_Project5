# from env 


from math import sqrt
from abc import ABC, abstractmethod

class Node(object):
    '''
    Class for searching nodes.

    Parameters
    ----------
    current: tuple
        current coordinate
    parent: tuple
        coordinate of parent node
    g: float
        path cost
    h: float
        heuristic cost

    Examples
    ----------
    >>> from env import Node
    >>> node1 = Node((1, 0), (2, 3), 1, 2)
    >>> node2 = Node((1, 0), (2, 5), 2, 8)
    >>> node3 = Node((2, 0), (1, 6), 3, 1)
    ...
    >>> node1 + node2
    >>> Node((2, 0), (2, 3), 3, 2)
    ...
    >>> node1 == node2
    >>> True
    ...
    >>> node1 != node3
    >>> True
    '''
    def __init__(self, current: tuple, parent: tuple, g: float, h: float) -> None:
        self.current = current
        self.parent = parent
        self.g = g
        self.h = h
    
    def __add__(self, node):
        return Node((self.current[0] + node.current[0], self.current[1] + node.current[1]), 
                     self.parent, self.g + node.g, self.h)

    def __eq__(self, node) -> bool:
        return self.current == node.current
    
    def __ne__(self, node) -> bool:
        return not self.__eq__(node)

    def __lt__(self, node) -> bool:
        return self.g + self.h < node.g + node.h or \
                (self.g + self.h == node.g + node.h and self.h < node.h)

    def __str__(self) -> str:
        return "----------\ncurrent:{}\nparent:{}\ng:{}\nh:{}\n----------" \
            .format(self.current, self.parent, self.g, self.h)

class Env(ABC):
    '''
    Class for building 2-d workspace of robots.

    Parameters
    ----------
    x_range: int
        x-axis range of enviroment
    y_range: int
        y-axis range of environmet

    Examples
    ----------
    >>> from utils import Env
    >>> env = Env(30, 40)
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range

    @property
    def grid_map(self) -> set:
        return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid(Env):
    '''
    Class for discrete 2-d grid map.
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        # allowed motions
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = None
        self.init()
    
    def init(self) -> None:
        '''
        Initialize grid map.
        '''
        x, y = self.x_range, self.y_range
        obstacles = set()

        # boundary of environment
        for i in range(x):
            obstacles.add((i, 0))
            obstacles.add((i, y - 1))
        for i in range(y):
            obstacles.add((0, i))
            obstacles.add((x - 1, i))

        # user-defined obstacles
        # for i in range(3, 15):
        #     obstacles.add((i, 10))
        # for i in range(7, 11):
        #     obstacles.add((3, i))
        # for i in range(4, 16):
        #     obstacles.add((i, 2))
        # for i in range(3, 7):
        #     obstacles.add((7, i))
        # for i in range(19, 27):
        #     obstacles.add((i, 12))
        # for i in range(2, 7):
        #     obstacles.add((20, i))
        # obstacles.add((21, 2))
        # obstacles.add((22, 2))
        # for i in range(14, 18):
        #     obstacles.add((5, i))
        #     obstacles.add((13, i))
        # for i in range(5, 9):
        #     obstacles.add((25, i))
        #     obstacles.add((26, i))
        # for i in range(15, 18):
        #     obstacles.add((23, i))
        #     obstacles.add((24, i))
        for i in range(10, 21):
            obstacles.add((i, 15))
        for i in range(15):
            obstacles.add((20, i))
        for i in range(15, 30):
            obstacles.add((30, i))
        for i in range(16):
            obstacles.add((40, i))

        self.obstacles = obstacles

    def update(self, obstacles):
        self.obstacles = obstacles 


class Map(Env):
    '''
    Class for continuous 2-d map.
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        '''
        Initialize map.
        '''
        x, y = self.x_range, self.y_range

        # boundary of environment
        self.boundary = [
            [0, 0, 1, y],
            [0, y, x, 1],
            [1, 0, x, 1],
            [x, 1, 1, y]
        ]

        # user-defined obstacles
        self.obs_rect = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]

        self.obs_circ = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

    def update(self, boundary, obs_circ, obs_rect):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect


#from planner 

import math
from abc import abstractmethod, ABC
# from .env import Env, Node
# from .plot import Plot

class Planner(ABC):
    def __init__(self, start: tuple, goal: tuple, env: Env) -> None:
        # plannig start and goal
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        # environment
        self.env = env
        # graph handler
        self.plot = Plot(start, goal, env)

    def dist(self, node1: Node, node2: Node) -> float:
        return math.hypot(node2.current[0] - node1.current[0], node2.current[1] - node1.current[1])
    
    def angle(self, node1: Node, node2: Node) -> float:
        return math.atan2(node2.current[1] - node1.current[1], node2.current[0] - node1.current[0])

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass

    @abstractmethod
    def run(self):
        '''
        Interface for running both plannig and animation.
        '''
        pass






### from plot.py 



import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# from .env import Env, Grid, Map, Node


class Plot:
    def __init__(self, start, goal, env: Env):
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()

    def animation(self, path, name, cost=None, expand=None, history_pose=None):
        name = name + "\ncost: " + str(cost) if cost else name
        self.plotEnv(name)
        if expand:
            self.plotExpand(expand)
        if history_pose:
            self.plotHistoryPose(history_pose)
        self.plotPath(path)
        plt.show()

    def plotEnv(self, name):
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

        if isinstance(self.env, Grid):
            obs_x = [x[0] for x in self.env.obstacles]
            obs_y = [x[1] for x in self.env.obstacles]
            plt.plot(obs_x, obs_y, "sk")

        if isinstance(self.env, Map):
            ax = self.fig.add_subplot()
            # boundary
            for (ox, oy, w, h) in self.env.boundary:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )
            # rectangle obstacles
            for (ox, oy, w, h) in self.env.obs_rect:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
            # circle obstacles
            for (ox, oy, r) in self.env.obs_circ:
                ax.add_patch(patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        plt.title(name)
        plt.axis("equal")

    def plotExpand(self, expand):
        if self.start in expand:
            expand.remove(self.start)
        if self.goal in expand:
            expand.remove(self.goal)

        count = 0
        if isinstance(self.env, Grid):
            for x in expand:
                count += 1
                plt.plot(x.current[0], x.current[1], color="#dddddd", marker='s')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event: [exit(0) if event.key == 'escape' else None])
                if count < len(expand) / 3:         length = 20
                elif count < len(expand) * 2 / 3:   length = 30
                else:                               length = 40
                if count % length == 0:             plt.pause(0.001)
        
        if isinstance(self.env, Map):
            for x in expand:
                count += 1
                if x.parent:
                    plt.plot([x.parent[0], x.current[0]], [x.parent[1], x.current[1]], 
                        color="#dddddd", linestyle="-")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)

        plt.pause(0.01)

    def plotPath(self, path) -> None:
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        plt.plot(path_x, path_y, linewidth='2', color='#13ae00')
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

    def plotAgent(self, pose: tuple, radius: float=1) -> None:
        x, y, theta = pose
        ref_vec = np.array([[radius / 2], [0]])
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]])
        end_pt = rot_mat @ ref_vec + np.array([[x], [y]])

        try:
            self.ax.artists.pop()
            for art in self.ax.get_children():
                if isinstance(art, matplotlib.patches.FancyArrow):
                    art.remove()
        except:
            pass

        self.ax.arrow(x, y, float(end_pt[0]) - x, float(end_pt[1]) - y,
                width=0.1, head_width=0.40, color="r")
        circle = plt.Circle((x, y), radius, color="r", fill=False)
        self.ax.add_artist(circle)

    def plotHistoryPose(self, history_pose):
        count = 0
        for pose in history_pose:
            if count < len(history_pose) - 1:
                plt.plot([history_pose[count][0], history_pose[count + 1][0]],
                    [history_pose[count][1], history_pose[count + 1][1]], c="r")
            count += 1
            self.plotAgent(pose)
            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'escape' else None])
            if count < len(history_pose) / 3:         length = 5
            elif count < len(history_pose) * 2 / 3:   length = 10
            else:                                     length = 20
            if count % length == 0:             plt.pause(0.01)

    def connect(self, name: str, func) -> None:
        self.fig.canvas.mpl_connect(name, func)

    def clean(self):
        plt.cla()

    def update(self):
        self.fig.canvas.draw_idle()

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl


# from sample search  

'''
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.17
'''
import numpy as np
from itertools import combinations
import math
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

# from utils import Env, Node, Plot, Planner

class SampleSearcher(Planner):
    '''
    Base class for planner based on sample searching.

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, delta: float=0.5) -> None:
        super().__init__(start, goal, env)
        # inflation bias
        self.delta = delta

    def isCollision(self, node1: Node, node2: Node) -> bool:
        '''
        Judge collision when moving from node1 to node2.

        Parameters
        ----------
        node1, node2: Node

        Return
        ----------
        collision: bool
            True if collision exists else False
        '''
        if self.isInsideObs(node1) or self.isInsideObs(node2):
            return True

        for rect in self.env.obs_rect:
            if self.isInterRect(node1, node2, rect):
                return True

        for circle in self.env.obs_circ:
            if self.isInterCircle(node1, node2, circle):
                return True

        return False

    def isInsideObs(self, node: Node) -> bool:
        '''
        Judge whether a node inside tht obstacles or not.

        Parameters
        ----------
        node1, node2: Node

        Return
        ----------
        inside: bool
            True if inside the obstacles else False
        '''
        x, y = node.current

        for (ox, oy, r) in self.env.obs_circ:
            if math.hypot(x - ox, y - oy) <= r + self.delta:
                return True

        for (ox, oy, w, h) in self.env.obs_rect:
            if 0 <= x - (ox - self.delta) <= w + 2 * self.delta \
                and 0 <= y - (oy - self.delta) <= h + 2 * self.delta:
                return True

        for (ox, oy, w, h) in self.env.boundary:
            if 0 <= x - (ox - self.delta) <= w + 2 * self.delta \
                and 0 <= y - (oy - self.delta) <= h + 2 * self.delta:
                return True

        return False

    def isInterRect(self, node1: Node, node2: Node, rect: list) -> bool:
        # obstacle and it's vertex
        ox, oy, w, h = rect
        vertex = [[ox - self.delta, oy - self.delta],
                  [ox + w + self.delta, oy - self.delta],
                  [ox + w + self.delta, oy + h + self.delta],
                  [ox - self.delta, oy + h + self.delta]]
        
        # node
        x1, y1 = node1.current
        x2, y2 = node2.current

        def cross(p1, p2, p3):
            x1 = p2[0] - p1[0]
            y1 = p2[1] - p1[1]
            x2 = p3[0] - p1[0]
            y2 = p3[1] - p1[1]
            return x1 * y2 - x2 * y1

        for v1, v2 in combinations(vertex, 2):
            # rapid repulsion
            if  max(x1, x2) >= min(v1[0], v2[0]) and \
                min(x1, x2) <= max(v1[0], v2[0]) and \
                max(y1, y2) >= min(v1[1], v2[1]) and \
                min(y1, y2) <= max(v1[1], v2[1]): 
                # cross
                if cross(v1, v2, node1.current) * cross(v1, v2, node2.current) <= 0 and \
                   cross(node1.current, node2.current, v1) * cross(node1.current, node2.current, v2) <= 0:
                    return True

        return False

    def isInterCircle(self, node1: Node, node2: Node, circle: list) -> bool:
        # obstacle
        ox, oy, r = circle

        # origin
        x, y = node1.current

        # direction
        dx = node2.current[0] - node1.current[0]
        dy = node2.current[1] - node1.current[1]
        d  = [dx, dy]
        d2 = np.dot(d, d)

        if d2 == 0:
            return False

        # projection
        t = np.dot([ox - x, oy - y], d) / d2
        if 0 <= t <= 1:
            shot = Node((x + t * dx, y + t * dy), None, None, None)
            center = Node((ox, oy), None, None, None)
            if self.dist(shot, center) <= r + self.delta:
                return True

        return False

### from rrt 


'''
@file: rrt.py
@breif: RRT motion planning
@author: Winter
@update: 2023.1.17
'''
import os, sys
import math
import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

# from .sample_search import SampleSearcher
# from utils import Env, Node

class RRT(SampleSearcher):
    '''
    Class for RRT motion planning.
    [1] Rapidly-Exploring Random Trees: A New Tool for Path Planning

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    max_dist: float
        Maximum expansion distance one step
    sample_num: int
        Maximum number of sample points
    goal_sample_rate: float
        heuristic sample

    Examples
    ----------
    >>> from utils import Map
    >>> from sample_search import RRT
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = RRT(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float, 
        sample_num: int, goal_sample_rate: float=0.05) -> None:
        super().__init__(start, goal, env)
        # Maximum expansion distance one step
        self.max_dist = max_dist
        # Maximum number of sample points
        self.sample_num = sample_num
        # heuristic sample
        self.goal_sample_rate = goal_sample_rate
        # Sampled list
        self.sample_list = [self.start]

    def __str__(self) -> str:
        return "Rapidly-exploring Random Tree(RRT)"

    def plan(self):
        '''
        RRT motion plan function.

        Return
        ----------
        cost: float
            path cost
        path: list
            planning path
        '''
        # main loop
        for _ in range(self.sample_num):
            # generate a random node in the map
            node_rand = self.generateRandomNode()

            # visited
            if node_rand in self.sample_list:
                continue
            
            # generate new node
            node_new = self.getNearest(self.sample_list, node_rand)
            if node_new:
                self.sample_list.append(node_new)
                dist = self.dist(node_new, self.goal)
                # goal found
                if dist <= self.max_dist and not self.isCollision(node_new, self.goal):
                    self.goal.parent = node_new.current
                    self.goal.g = node_new.g + self.dist(self.goal, node_new)
                    self.sample_list.append(self.goal)
                    return self.extractPath(self.sample_list)
        return 0, None

    def run(self) -> None:
        '''
        Running both plannig and animation.
        '''
        cost, path = self.plan()
        self.plot.animation(path, str(self), cost, self.sample_list)

    def generateRandomNode(self) -> Node:
        '''
        Generate a random node to extend exploring tree.

        Return
        ----------
        node: Node
            a random node based on sampling
        '''
        if np.random.random() > self.goal_sample_rate:
            current = (np.random.uniform(self.delta, self.env.x_range - self.delta),
                    np.random.uniform(self.delta, self.env.y_range - self.delta))
            return Node(current, None, 0, 0)
        return self.goal

    def getNearest(self, node_list: list, node: Node) -> Node:
        '''
        Get the node from `node_list` that is nearest to `node`.

        Parameters
        ----------
        node_list: list
            exploring list
        node: Node
            currently generated node

        Return
        ----------
        node: Node
            nearest node 
        '''
        # find nearest neighbor
        dist = [self.dist(node, nd) for nd in node_list]
        node_near = node_list[int(np.argmin(dist))]

        # regular and generate new node
        dist, theta = self.dist(node_near, node), self.angle(node_near, node)
        dist = min(self.max_dist, dist)
        node_new = Node((node_near.current[0] + dist * math.cos(theta),
                        (node_near.current[1] + dist * math.sin(theta))),
                         node_near.current, node_near.g + dist, 0)
        
        # obstacle check
        if self.isCollision(node_new, node_near):
            return None
        return node_new

    def extractPath(self, closed_set):
        '''
        Extract the path based on the CLOSED set.

        Parameters
        ----------
        closed_set: list
            CLOSED set

        Return
        ----------
        cost: float
            the cost of planning path
        path: list
            the planning path
        '''
        node = closed_set[closed_set.index(self.goal)]
        path = [node.current]
        cost = node.g
        while node != self.start:
            node_parent = closed_set[closed_set.index(Node(node.parent, None, None, None))]
            node = node_parent
            path.append(node.current)

        return cost, path
    

###RRT STAR 

class RRTStar(RRT):
    '''
    Class for RRT-Star motion planning.
    [1] Sampling-based algorithms for optimal motion planning

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    max_dist: float
        Maximum expansion distance one step
    sample_num: int
        Maximum number of sample points
    r: float
        optimization radius
    goal_sample_rate: float
        heuristic sample

    Examples
    ----------
    >>> from utils import Map
    >>> from sample_search import RRTStar
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = RRTStar(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float,
                 sample_num: int, r: float, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # optimization radius
        self.r = r
    
    def __str__(self) -> str:
        return "RRT*"
    
    def getNearest(self, node_list: list, node: Node) -> Node:
        '''
        Get the node from `node_list` that is nearest to `node` with optimization.

        Parameters
        ----------
        node_list: list
            exploring list
        node: Node
            currently generated node

        Return
        ----------
        node: Node
            nearest node 
        '''
        node_new = super().getNearest(node_list, node)
        if node_new:
            #  rewire optimization
            for node_n in node_list:
                #  inside the optimization circle
                new_dist = self.dist(node_n, node_new)
                if new_dist < self.r:
                    cost = node_n.g + new_dist
                    #  update new sample node's cost and parent
                    if node_new.g > cost and not self.isCollision(node_n, node_new):
                        node_new.parent = node_n.current
                        node_new.g = cost
                    else:
                        #  update nodes' cost inside the radius
                        cost = node_new.g + new_dist
                        if node_n.g > cost and not self.isCollision(node_n, node_new):
                            node_n.parent = node_new.current
                            node_n.g = cost
                else:
                    continue
            return node_new
        else:
            return None 
        

## from informed 
import os, sys
import numpy as np
from functools import partial
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

# from .rrt_star import RRTStar
# from utils import Env, Node

class ellipse:
    '''
    Ellipse sampling.
    '''    
    @staticmethod
    def transform(a: float, c: float, p1: tuple, p2: tuple) -> np.ndarray:
        # center
        center_x = (p1[0] + p2[0]) / 2
        center_y = (p1[1] + p2[1]) / 2

        # rotation
        theta = - np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

        # transform
        b = np.sqrt(a ** 2 - c ** 2)
        T = np.array([[ a * np.cos(theta), b * np.sin(theta), center_x],
                      [-a * np.sin(theta), b * np.cos(theta), center_y],
                      [                 0,                 0,        1]])
        return T

class InformedRRT(RRTStar):
    '''
    Class for Informed RRT* motion planning.
    [1] Optimal Sampling-based Path Planning Focused via Direct
        Sampling of an Admissible Ellipsoidal heuristic

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    max_dist: float
        Maximum expansion distance one step
    sample_num: int
        Maximum number of sample points
    r: float
        optimization radius
    goal_sample_rate: float
        heuristic sample

    Examples
    ----------
    >>> from utils import Map
    >>> from sample_search import InformedRRT
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = InformedRRT(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float,
                 sample_num: int, r: float, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # optimization radius
        self.r = r
        # best planning cost
        self.c_best = float("inf")
        # distance between start and goal
        self.c_min = self.dist(self.start, self.goal)
        # ellipse sampling
        self.transform = partial(ellipse.transform, c=self.c_min / 2, p1=start, p2=goal)
    
    def __str__(self) -> str:
        return "Informed RRT*"

    def plan(self):
        '''
        Informed-RRT* motion plan function.

        Return
        ----------
        cost: float
            path cost
        path: list
            planning path
        '''
        # generate a random node in the map
        node_rand = self.generateRandomNode()

        # visited
        if node_rand in self.sample_list:
            return 0, None

        # generate new node
        node_new = self.getNearest(self.sample_list, node_rand)
        if node_new:
            self.sample_list.append(node_new)
            dist = self.dist(node_new, self.goal)
            # goal found
            if dist <= self.max_dist and not self.isCollision(node_new, self.goal):
                self.goal.parent = node_new.current
                self.goal.g = node_new.g + self.dist(self.goal, node_new)
                self.sample_list.append(self.goal)
                return self.extractPath(self.sample_list)
        return 0, None

    def run(self) -> None:
        '''
        Running both plannig and animation.
        '''
        best_cost, best_path = float("inf"), None

        # main loop
        for i in range(self.sample_num):
            cost, path = self.plan()
            # update
            if path and cost < best_cost:
                self.c_best = best_cost = cost
                best_path = path
            # animation
            if  i % 30 == 0:
                self.animation(best_path, best_cost)
        plt.show()

    def generateRandomNode(self) -> Node:
        '''
        Generate a random node to extend exploring tree.

        Return
        ----------
        node: Node
            a random node based on sampling
        '''
        # ellipse sample
        if self.c_best < float("inf"):
            while True:
                # unit ball sample
                p = np.array([.0, .0, 1.])
                while True:
                    x, y = np.random.uniform(-1, 1), np.random.uniform(-1, 1)
                    if x ** 2 + y ** 2 < 1:
                        p[0], p[1] = x, y
                        break
                # transform to ellipse
                p_star = self.transform(self.c_best / 2) @ p.T
                if self.delta <= p_star[0] <= self.env.x_range - self.delta and \
                   self.delta <= p_star[1] <= self.env.y_range - self.delta:
                    return Node((p_star[0], p_star[1]), None, 0, 0)
        # random sample
        else:
            return super().generateRandomNode()

    def animation(self, path, cost):
        self.plot.clean()
        name = str(self) + "\ncost: " + str(cost)
        self.plot.plotEnv(name)
        for x in self.sample_list:
            if x.parent:
                plt.plot([x.parent[0], x.current[0]], [x.parent[1], x.current[1]], 
                    color="#dddddd", linestyle="-")
        if self.c_best < float("inf"):
            self.drawEllipse()
        if path:
            self.plot.plotPath(path)
        plt.pause(0.01)

    def drawEllipse(self):
        t = np.arange(0, 2 * np.pi + 0.1, 0.1)
        x = [np.cos(it) for it in t]
        y = [np.sin(it) for it in t]
        z = [1 for _ in t]
        fx = self.transform(self.c_best / 2) @ np.array([x, y, z])
        plt.plot(fx[0, :], fx[1, :], linestyle='--', color='darkorange', linewidth=2)
    
# main code 
# from utils import Grid, Map, SearchFactory
# from local_planner import DWA
# from sample_search import RRT

if __name__ == '__main__':
    start = (18, 8)
    goal = (37, 18)
    env = Map(51, 31)

    planner = InformedRRT(start, goal, env, max_dist=0.5
                      ,sample_num=1200,r=12)

    # animation
    planner.run()
