import numpy as np
import cv2
from map import *
import time
import json

if __name__ == "__main__":

    f = open('config.json',)
    
    data = json.load(f)
    
    for param in data['parameters']:
        height = param['height']
        width = param['width']
        start_pose = param['start_pose']
        goal_pose = param['goal_pose']
        step_size = param['step_size']
        search_radius = param['search_radius']
        ITERATIONS = param['ITERATIONS']
        show_edges = param['show_edges']
    
    f.close()

    map = Map(height, width, step_size, start_pose, goal_pose)

    map.show_edges = show_edges

    for i in range(0,width,50):
        for j in range(0,height,50):
            map.add_obstacle(i,j,30,30)

    x_new = Node(map.start.x,map.start.y)

    map.display_map(x_new)

    while(map.euclidean_distance(x_new,map.goal) > map.step_size):

        if(map.solution_found):
            map.c_best = map.x_soln.sort()[0]
        
        x_rand = map.sample(start_pose,goal_pose,map.c_best)

        nearest_node_found, x_nearest, cost = map.nearest_node(x_rand)

        if(nearest_node_found):
            pass
        else:
            continue

        x_new, cost_new = map.steer(x_nearest, x_rand, cost)
        
        edge = Edge(x_new, x_nearest, cost_new)
        
        if(map.collision_free(x_nearest, x_new)):
            x_new.parent = x_nearest
            x_new.cost = x_new.parent.cost + cost_new
            map.nodes.append(x_new)

            edge.node_1 = x_new
            edge.node_2 = x_nearest
            edge.cost = cost_new

            map.edges.append(edge)

        map.display_map(x_rand)

    map.solution_found = True

    map.goal.parent = x_new
    map.goal.cost = x_new.cost + map.euclidean_distance(x_new,map.goal)
    map.nodes.append(map.goal)
    cost_final = map.euclidean_distance(x_new,map.goal)
    final_edge = Edge(x_new, map.goal, cost_final)
    map.edges.append(final_edge)

    map.display_map(x_rand)

    map.display_map(x_rand,best_path_found=True)
    cv2.waitKey(0)