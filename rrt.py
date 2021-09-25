from map import Map
import utils
import cv2
import time

height = 500
width = 500
start_pose = [10,10]
goal_pose = [430,430]
step_size = 20.0


if __name__ == "__main__":
    map = Map(height,width,step_size)

    map.add_start(start_pose[0],start_pose[1]) # add start
    map.add_goal(goal_pose[0],goal_pose[1]) # add goal

    for i in range(50,100):
        for j in range(50,100):
            map.add_obstacle(i,j)
    
    for i in range(150,200):
        for j in range(150,200):
            map.add_obstacle(i,j)

    for i in range(250,300):
        for j in range(250,300):
            map.add_obstacle(i,j)
    
    for i in range(350,400):
        for j in range(350,400):
            map.add_obstacle(i,j)


    # sample random point
    rand_x, rand_y = map.sample_point()
    random_point = [rand_x,rand_y]

    # terminate if goal is near
    # while(utils.euclidean_distance(random_point,goal_pose) > step_size):
    while(True):

        s_time = time.time()

        # find nearest node
        found_nearest_node, nearest_node = map.find_nearest(rand_x, rand_y)

        print("Finding Nearest Node : ",time.time() - s_time)

        # if no nearest node found, then sample again
        if(found_nearest_node == False):
            rand_x, rand_y = map.sample_point()
            random_point = [rand_x,rand_y]
            continue

        # take a step towards the node
        step_to_node = map.take_step(nearest_node,random_point)


        print("Taking Step to Node : ",time.time() - s_time)

        # create node at sampled random point, and edge between that and nearest node
        map.add_node(step_to_node[0],step_to_node[1])
        map.add_edge(map.nodes.index(nearest_node),len(map.nodes)-1)
        # if(utils.euclidean_distance(step_to_node,goal_pose) < step_size):
        #     break

        found_goal_nearby, node_near_goal = map.find_nearest(goal_pose[0],goal_pose[1])

        if(found_goal_nearby and utils.euclidean_distance(node_near_goal,goal_pose) < step_size):
            map.add_node(goal_pose[0],goal_pose[1])
            break

        print("Adding node and edge : ",time.time() - s_time)

        rand_x, rand_y = map.sample_point()
        random_point = [rand_x,rand_y]

        map.display_map()
        print("Display : ",time.time() - s_time)

    """
    faulty destination check
    """
    cv2.waitKey(0)
    print("Sampled Destination")
    # create node at sampled random point, and edge between that and nearest node
    map.add_node(goal_pose[0],goal_pose[1])
    print(map.nodes)
    map.add_edge(len(map.nodes)-2,len(map.nodes)-1)
    map.display_map()
    print("Display : ",time.time() - s_time)
        
    cv2.waitKey(0)