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

from utils import Map, Node, Plot

class SampleSearcher():
    '''
    Base class for planner based on sample searching.

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Map
        environment
    '''
    def __init__(self, start: tuple, goal: tuple, env: Map, delta: float=0.5) -> None:
        # super().__init__(start, goal, env)
        # inflation bias
        self.delta = delta
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        # environment
        self.env = env
        # graph handler
        self.plot = Plot(start, goal, env)
    

    def dist(self, node1: Node, node2: Node) -> float:
        return math.hypot(node2.x - node1.x, node2.y - node1.y)
    
    def angle(self, node1: Node, node2: Node) -> float:
        return math.atan2(node2.y - node1.y, node2.x - node1.x)

    def collsion(self, node1: Node, node2: Node) -> bool:
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

    