import numpy as np
import cv2
from map import *
import time
import tqdm

if __name__ == "__main__":

    start_pose = [60,60]
    goal_pose = [350,350]
    height = 400
    width = 400
    step_size = 22
    search_radius = 27.0
    ITERATIONS = 3000

    if(search_radius < step_size):
        print("search radius should be > step_size")

    map = Map(height, width, step_size, start_pose, goal_pose)

    map.set_node_cost(map.start)

    # for i in range(0,width,40):
    #     for j in range(0,height,40):
    #         map.add_obstacle(i,j,20,20)

    # map.add_obstacle(200,200,100,100)
    # map.add_obstacle(150,200,250,40)
    # map.add_obstacle(2,260,200,40)

    x_new = Node(map.start.x,map.start.y)

    _ = map.display_map(x_new)

    while(map.euclidean_distance(x_new,map.goal) > search_radius):

        if(map.solution_found):
            pass
        
        x_rand = map.sample(start_pose,goal_pose,np.inf)

        nearest_node_found, x_nearest, cost = map.nearest_node(x_rand)

        if(nearest_node_found):
            pass
        else:
            continue

        x_new, cost_new = map.steer(x_nearest, x_rand, cost)

        if(not map.is_valid(x_new)):
            continue
        if(map.is_in_obstacle(x_new)):
            continue

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

        _ = map.display_map(x_rand)

    map.solution_found = True

    map.goal.parent = x_new
    map.goal.cost = x_new.cost + map.euclidean_distance(x_new,map.goal)
    map.nodes.append(map.goal)
    cost_final = map.euclidean_distance(x_new,map.goal)
    final_edge = Edge(x_new, map.goal, cost_final)
    map.edges.append(final_edge)

    c_best = map.display_map(x_rand)

    c_best = map.display_map(x_rand,best_path_found=True)
    print("c_best : ",c_best)

    print("refining")
    for i in tqdm.tqdm(range(ITERATIONS)):

        x_rand = map.informed_sample(map.start, map.goal, c_best)

        nearest_node_found, x_nearest, cost = map.nearest_node(x_rand)

        if(nearest_node_found):
            pass
        else:
            continue

        x_new, cost_new = map.steer(x_nearest, x_rand, cost)
        if(not map.is_valid(x_new)):
            continue
        if(map.is_in_obstacle(x_new)):
            continue

        map.set_node_cost(x_new)

        nodes_in_radius = map.get_nodes_in_radius(search_radius, x_new)

        # edge = Edge(x_new, x_nearest, cost_new)

        map.rewire(x_new,nodes_in_radius)
        cv2.waitKey(0)

    c_best = map.display_converged_map(x_rand)
    cv2.waitKey(0)