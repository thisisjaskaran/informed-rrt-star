import numpy as np
import cv2
from map import *
import time

if __name__ == "__main__":

    start_pose = [20,30]
    goal_pose = [410,440]
    height = 500
    width = 700
    step_size = 7

    map = Map(height, width, step_size, start_pose, goal_pose)

    # map.add_obstacle(50,150,280,280)

    for i in range(0,width,30):
        for j in range(0,height,30):
            map.add_obstacle(i,j,10,10)

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

        print(f"x_rand : ({x_rand.x},{x_rand.y}) x_nearest : ({x_nearest.x},{x_nearest.y}) x_new : ({x_new.x},{x_new.y}) cost_new : {cost_new}")
        
        edge = Edge(x_new, x_nearest, cost_new)
        
        if(map.collision_free(x_nearest, x_new)):
            x_new.parent = x_nearest
            map.nodes.append(x_new)

            edge.node_1 = x_new
            edge.node_2 = x_nearest
            edge.cost = cost_new

            map.edges.append(edge)

        map.display_map(x_rand)
        # time.sleep(1)

    map.solution_found = True

    map.goal.parent = x_new
    map.nodes.append(map.goal)
    cost_final = map.euclidean_distance(x_new,map.goal)
    final_edge = Edge(x_new, map.goal, cost_final)
    map.edges.append(final_edge)

    map.display_map(x_rand)

    map.display_map(x_rand,best_path_found=True)
    cv2.waitKey(0)