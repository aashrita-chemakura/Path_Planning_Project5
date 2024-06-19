'''
@file: global_planner.py
@breif: global planner application entry
@author: Winter
@update: 2023.3.2
'''
from utils import Map
# from local_planner import DWA
from sample_search import InformedRRT,RRT,RRTStar,ImprovedRRTStar
if __name__ == '__main__':
    start = (5, 45)
    goal = (45, 15)
    env = Map(51, 51)

    planner = ImprovedRRTStar(start,goal, env, max_dist=0.5,
                      sample_num=10000,r=12,c_best=62)

    # planner = InformedRRT(start,goal, env, max_dist=0.5,
    #                   sample_num=10000,r=12)

    # planner = RRTStar(start,goal, env, max_dist=0.5,
    #                   sample_num=10000,r=12)

    # planner = RRT(start,goal, env, max_dist=0.5,
    #                   sample_num=10000)
                                      

    # animation
    planner.run()