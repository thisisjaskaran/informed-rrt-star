import numpy as np
import cv2
from map import *
import time

if __name__ == "__main__":

    start_pose = [200,200]
    goal_pose = [380,380]
    height = 400
    width = 400
    step_size = 10
    search_radius = 40.0

    if(search_radius < step_size):
        print("search radius should be > step_size")

    map = Map(height, width, step_size, start_pose, goal_pose)

    map.set_node_cost(map.start)

    # for i in range(0,width,50):
    #     for j in range(0,height,50):
    #         map.add_obstacle(i,j,30,30)

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

        map.set_node_cost(x_new)

        nodes_in_radius = map.get_nodes_in_radius(search_radius, x_new)

        edge = Edge(x_new, x_nearest, cost_new)

        if(map.first_sample):
            x_new.parent = map.start
            x_new.cost = map.euclidean_distance(x_new,map.start)
            map.nodes.append(x_new)
            
            edge.node_1 = map.start
            edge.node_2 = x_new
            edge.cost = x_new.cost
            map.edges.append(edge)

            map.first_sample = False
            
        else:
            map.rewire(x_new,nodes_in_radius)

        map.display_map(x_rand)

        # cv2.waitKey(0)

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