'''
@file: env.py
@breif: 2-dimension environment and graph node
@author: Winter
@update: 2023.1.13
'''
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
        self.x=current[0]
        self.y=current[1]
        self.parent = parent
        self.g = g
        self.h = h
    
    # def __add__(self, node):
    #     return Node((self.current[0] + node.current[0], self.current[1] + node.current[1]), 
    #                  self.parent, self.g + node.g, self.h)
    def __add__(self, node):
        return Node((self.x + node.x, self.y + node.y), 
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

# class Env(ABC):
#     '''
#     Class for building 2-d workspace of robots.

#     Parameters
#     ----------
#     x_range: int
#         x-axis range of enviroment
#     y_range: int
#         y-axis range of environmet

#     Examples
#     ----------
#     >>> from utils import Env
#     >>> env = Env(30, 40)
#     '''
#     def __init__(self, x_range: int, y_range: int) -> None:
#         # size of environment
#         self.x_range = x_range  
#         self.y_range = y_range

#     # @abstractmethod
#     # def init(self) -> None:
#     #     pass



class Map():
    '''
    Class for continuous 2-d map.
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        # super().__init__(x_range, y_range)
        self.x_range = x_range  
        self.y_range = y_range
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


        #map one
        # self.obs_rect = [
        #     # [14, 12, 8, 2],
        #     [10,10,30,10],
        # ]

        #map two
        self.obs_rect = [
            [10,31,2,20], #tall rect
            [1,18,25,2], # long rect 
            [12,8,2,10],
            [25,30,2,10] , #second vert rect 
            [24,40,27,2],
            [32,35,19,2],
            [35,7,2,23], #third vect rect 
            [37,9,9,2],
            [17,26,2,4]
        ]

   

    # def update(self, boundary, obs_circ, obs_rect):
    #     self.boundary = boundary if boundary else self.boundary
    #     self.obs_circ = obs_circ if obs_circ else self.obs_circ
    #     self.obs_rect = obs_rect if obs_rect else self.obs_rect
