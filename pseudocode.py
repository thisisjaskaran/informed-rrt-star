import numpy as np
import cv2
import random
import time

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None

class Edge:
    def __init__(self,node_1,node_2,cost):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = cost

class Map:
    def __init__(self, height, width, step_size, start, goal):
        self.height = height
        self.width = width

        self.step_size = step_size
        self.map =  np.ones((self.height,self.width,3), np.uint8) * 255 # numpy array

        self.start = Node(start[0],start[1]) # object of class Node
        self.goal = Node(goal[0],goal[1]) # object of class Node

        self.obstacle_list = [] # list of tuple (x,y)
        self.nodes = [self.start] # list of Node objects
        self.edges = [] # list of Edge objects
        self.x_soln = []

        self.solution_found = False
        self.c_best = np.inf
        self.bot_safety_distance = 1.0

    def add_obstacle(self,x,y,width,height):
        for i in range(x,x+width):
            for j in range(y,y+height):
                self.map[x,y] = (0,0,0)
                self.obstacle_list.append((x,y))
    
    def display_map(self,best_path_found = False):
        img = self.map
        img = cv2.circle(img,(self.start.x,self.start.y),5,(0,255,0),-1)
        img = cv2.circle(img,(self.goal.x,self.goal.y),5,(0,0,255),-1)

        for edge in self.edges:
            node_1 = (edge.node_1.x,edge.node_1.y)
            node_2 = (edge.node_2.x,edge.node_2.y)
            img = cv2.line(img,node_1,node_2,(255,0,0),2)

        if(best_path_found):
            curr_node = self.goal
            while(curr_node is not None):
                self.x_soln.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(self.x_soln)-1):
                node_1 = (self.x_soln[i].x,self.x_soln[i].y)
                node_2 = (self.x_soln[i+1].x,self.x_soln[i+1].y)
                img = cv2.line(img,node_1,node_2,(0,0,255),4)

        cv2.imshow("img",img)
        cv2.waitKey(10)
    
    def euclidean_distance(self, node_1, node_2):
        """
        Returns euclidean distance between 2 nodes
        Inputs : Node node_1, Node node_2
        Output : distance
        """
        return np.sqrt( (node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
    
    def sample(self, x_start, x_goal, c_max):
        """
        Returns random sample
        Inputs : Node x_start, Node x_goal, max_cost
        Output : Node random_node
        """
        _1 = x_start
        _2 = x_goal
        _3 = c_max

        random_x = random.randint(0,self.width)
        random_y = random.randint(0,self.height)

        random_node = Node(random_x, random_y)

        return random_node
    
    def nearest_node(self, x_rand):
        """
        Return nearest node to random sample
        Inputs : Node x_nearest, Node random_node
        Output : Node nearest_node, cost
        """
        cost = np.inf
        nearest_node_to_sample = None
        found_nearest_node = False

        for node_ in self.nodes:
            if(self.euclidean_distance(x_rand,node_) < cost):
                cost = self.euclidean_distance(x_rand,node_)
                nearest_node_to_sample = node_
                found_nearest_node = True

        if(nearest_node_to_sample is not None):
            return found_nearest_node, nearest_node_to_sample, cost
        else:
            return found_nearest_node, x_rand, 0
    
    def steer(self, x_nearest, x_rand, cost):
        """
        Return nearest node to random sample
        Inputs : Node x_nearest, Node random_node
        Output : Node stepped_node, cost
        """
        det = self.euclidean_distance(x_nearest, x_rand)

        if(det <= self.step_size):
            # x_rand.parent = x_nearest
            return x_rand, cost
        else:
            step_x = int(x_nearest.x + (x_rand.x - x_nearest.x) * self.step_size / det)
            step_y = int(x_nearest.y + (x_rand.y - x_nearest.y) * self.step_size / det)
            
            stepped_node = Node(step_x, step_y)
            # stepped_node.parent = x_nearest

            cost = self.euclidean_distance(stepped_node, x_nearest)
            
            return stepped_node, cost
    
    def collision_free(self, x_nearest, x_new):
        """
        Return nearest node to random sample
        Inputs : Node x_nearest, Node x_new
        Output : Node stepped_node, cost
        """
        return 1
    
    def node_idx_in_edge(self,edge,node):
        if(node == edge.node_1):
            return 1
        else:
            return 2
    
    def print_nodes(self):
        for node in self.nodes:
            print(node.x,node.y)

    def print_best_path(self):
        for node in self.x_soln:
            print(node.x,node.y)

if __name__ == "__main__":

    start_pose = [20,30]
    goal_pose = [400,450]
    height = 500
    width = 600
    step_size = 20

    map = Map(height, width, step_size, start_pose, goal_pose)

    map.add_obstacle(50,50,20,20)

    x_new = Node(map.start.x,map.start.y)

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

        # print(f"x_rand : ({x_rand.x},{x_rand.y}) x_nearest : ({x_nearest.x},{x_nearest.y}) x_new : ({x_new.x},{x_new.y}) cost_new : {cost_new}")
        
        edge = Edge(x_new, x_nearest, cost_new)
        
        if(map.collision_free(x_nearest, x_new)):
            x_new.parent = x_nearest
            map.nodes.append(x_new)

            edge.node_1 = x_new
            edge.node_2 = x_nearest
            edge.cost = cost_new

            map.edges.append(edge)

        map.display_map()

    map.solution_found = True

    map.goal.parent = x_new
    map.nodes.append(map.goal)
    cost_final = map.euclidean_distance(x_new,map.goal)
    final_edge = Edge(x_new, map.goal, cost_final)
    map.edges.append(final_edge)
    map.print_best_path()

    map.display_map()

    map.display_map(best_path_found=True)
    cv2.waitKey(0)