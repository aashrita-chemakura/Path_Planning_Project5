"""
Plot tools 2D
@author: huiming zhou
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from .env import Map, Node


class Plot:
    def __init__(self, start, goal, env: Map):
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()

    def animation(self, path, name, cost=None, expand=None):
        name = name + "\ncost: " + str(cost) if cost else name
        self.plotEnv(name)
        if expand:
            self.plotExpand(expand)
     
        self.plotPath(path)
        plt.show()

    def plotEnv(self, name):
        # plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        # plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

        plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
        plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")


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
           

        plt.title(name)
        plt.axis("equal")

    def plotExpand(self, expand):
        if self.start in expand:
            expand.remove(self.start)
        if self.goal in expand:
            expand.remove(self.goal)

        count = 0
        
        
        if isinstance(self.env, Map):
            for x in expand:
                count += 1
                if x.parent:
                    plt.plot([x.parent[0], x.x], [x.parent[1], x.y], 
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
        # plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        # plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")
        plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
        plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")

   

    # def connect(self, name: str, func) -> None:
    #     self.fig.canvas.mpl_connect(name, func)

    def clean(self):
        plt.cla()

    # def update(self):
    #     self.fig.canvas.draw_idle()

 