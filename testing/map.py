# x is width, y is height
# just do x,y everywhere

import cv2
import numpy as np
import random
import math
import time

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

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
        self.map =  np.ones((self.height,self.width,3), np.uint8) * 255

        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])

        self.obstacle_list = []
        self.nodes = [self.start]
        self.edges = []
        self.x_soln = []

        self.solution_found = False
        self.bot_safety_distance = 1.0
        self.first_sample = True

        self.major_axis = 0.0
        self.minor_axis = 0.0

        self.best_cost_for_informed = np.inf
        self.show_edges = 0
        self.show_sample = 0
        self.show_ellipse = 0

    def add_obstacle(self,x,y,width,height,clearance):
        flag = 1
        for i in range(x - clearance , x + width + clearance):
            for j in range(y - clearance , y + height + clearance):
                if(i == self.start.x and j == self.start.y):
                    flag = 0
                    break
                if(i == self.goal.x and j == self.goal.y):
                    flag = 0
                    break
        
        if(flag):
            for i in range(x,x+width):
                for j in range(y,y+height):
                    self.map[i,j,0] = 0
                    self.map[i,j,1] = 0
                    self.map[i,j,2] = 0
                    self.obstacle_list.append((j,i))
    
    def remove_anomalies(self,clearance):
        for i in range((-1)*clearance,clearance):
            for j in range((-1)*clearance,clearance):
                self.map[self.start.x + i, self.start.x + j,0] = 255
                self.map[self.start.x + i, self.start.x + j,1] = 255
                self.map[self.start.x + i, self.start.x + j,2] = 255

                self.map[self.goal.x + i, self.goal.x + j,0] = 255
                self.map[self.goal.x + i, self.goal.x + j,1] = 255
                self.map[self.goal.x + i, self.goal.x + j,2] = 255
    
    def display_map(self,x_rand,best_path_found = False):
        best_cost = 0.0
        # img = self.map
        img = np.ones((self.height,self.width,3), np.uint8) * 255 # numpy array
        img = cv2.circle(img,(self.start.x,self.start.y),5,(0,255,0),-1)
        img = cv2.circle(img,(self.goal.x,self.goal.y),5,(0,0,255),-1)
        if(self.show_sample):
            img = cv2.circle(img,(x_rand.x,x_rand.y),5,(0,0,0),-1)

        for edge in self.edges:
            node_1 = (edge.node_1.x,edge.node_1.y)
            node_2 = (edge.node_2.x,edge.node_2.y)
            if(self.show_edges):
                img = cv2.line(img,node_1,node_2,(255,0,0),1)

        if(best_path_found):
            curr_node = self.goal
            while(curr_node is not None):
                self.x_soln.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(self.x_soln)-1):
                node_1 = (self.x_soln[i].x,self.x_soln[i].y)
                node_2 = (self.x_soln[i+1].x,self.x_soln[i+1].y)
                img = cv2.line(img,node_1,node_2,(0,0,255),4)

        for obstacle in self.obstacle_list:
            img[obstacle[0],obstacle[1]] = (0,0,0)

        for node in self.nodes:
            curr_node = node
            path = []
            while(curr_node is not None):
                path.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(path)-1):
                node_1 = (path[i].x,path[i].y)
                node_2 = (path[i+1].x,path[i+1].y)
                if(self.show_edges):
                    img = cv2.line(img,node_1,node_2,(255,0,0),1)
                node_1_N = Node(node_1[0],node_1[1])
                node_2_N = Node(node_2[0],node_2[1])
                best_cost += self.euclidean_distance(node_1_N,node_2_N)

        cv2.imshow("img",img)
        cv2.waitKey(2)

        return best_cost
    
    def display_converged_map(self,x_rand):
        best_cost = 0.0

        img = np.ones((self.height,self.width,3), np.uint8) * 255 # numpy array
        img = cv2.circle(img,(self.start.x,self.start.y),5,(0,255,0),-1)
        img = cv2.circle(img,(self.goal.x,self.goal.y),5,(0,0,255),-1)
        if(self.show_sample):
            img = cv2.circle(img,(x_rand.x,x_rand.y),5,(0,0,0),-1)

        for edge in self.edges:
            node_1 = (edge.node_1.x,edge.node_1.y)
            node_2 = (edge.node_2.x,edge.node_2.y)
            if(self.show_edges):
                img = cv2.line(img,node_1,node_2,(255,0,0),1)

        curr_node = self.goal

        try:
            while(curr_node.parent is not self.start):
                node_1 = (curr_node.x,curr_node.y)
                node_2 = (curr_node.parent.x,curr_node.parent.y)

                img = cv2.line(img,node_1,node_2,(0,0,255),2)
                curr_node = curr_node.parent
            node_1 = (curr_node.x,curr_node.y)
            node_2 = (curr_node.parent.x,curr_node.parent.y)
            img = cv2.line(img,node_1,node_2,(0,0,255),2)
        except:
            pass

        for obstacle in self.obstacle_list:
            img[obstacle[0],obstacle[1]] = (0,0,0)

        for node in self.nodes:
            curr_node = node
            path = []
            while(curr_node is not None):
                path.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(path)-1):
                node_1 = (path[i].x,path[i].y)
                node_2 = (path[i+1].x,path[i+1].y)
                if(self.show_edges):
                    img = cv2.line(img,node_1,node_2,(255,0,0),1)
                node_1_N = Node(node_1[0],node_1[1])
                node_2_N = Node(node_2[0],node_2[1])
                best_cost += self.euclidean_distance(node_1_N,node_2_N)

        cv2.imshow("img",img)
        cv2.waitKey(2)

        return best_cost

    def display_informed_converged_map(self,x_rand, final = False):
        best_cost = 0.0

        img = np.ones((self.height,self.width,3), np.uint8) * 255 # numpy array
        img = cv2.circle(img,(self.start.x,self.start.y),5,(0,255,0),-1)
        img = cv2.circle(img,(self.goal.x,self.goal.y),5,(0,0,255),-1)
        if(self.show_sample):
            img = cv2.circle(img,(x_rand.x,x_rand.y),5,(0,0,0),-1)

        if(not final):
            for edge in self.edges:
                node_1 = (edge.node_1.x,edge.node_1.y)
                node_2 = (edge.node_2.x,edge.node_2.y)
                if(self.show_edges):
                    img = cv2.line(img,node_1,node_2,(255,0,0),1)

        for obstacle in self.obstacle_list:
            img[obstacle[0],obstacle[1]] = (0,0,0)

        if(not final):
            for node in self.nodes:
                curr_node = node
                path = []
                while(curr_node is not None):
                    path.append(curr_node)
                    curr_node = curr_node.parent
            
                for i in range(len(path)-1):
                    node_1 = (path[i].x,path[i].y)
                    node_2 = (path[i+1].x,path[i+1].y)
                    if(self.show_edges):
                        img = cv2.line(img,node_1,node_2,(255,0,0),1)
                    node_1_N = Node(node_1[0],node_1[1])
                    node_2_N = Node(node_2[0],node_2[1])
                    best_cost += self.euclidean_distance(node_1_N,node_2_N)

            if(self.show_ellipse):
                center_coordinates = (int((self.start.x + self.goal.x)/2) , int((self.start.y + self.goal.y)/2))
                ellipse_angle =  self.nodes_slope(self.start,self.goal) * 180 / np.pi

                img = cv2.ellipse(  img, center_coordinates, (int(self.major_axis),int(self.minor_axis)),
                                    ellipse_angle, 0, 360, (128,128,128), 2)
            
        curr_node = self.goal
        
        try:
            while(curr_node.parent is not self.start):
                node_1 = (curr_node.x,curr_node.y)
                node_2 = (curr_node.parent.x,curr_node.parent.y)

                img = cv2.line(img,node_1,node_2,(0,0,255),2)
                curr_node = curr_node.parent
            node_1 = (curr_node.x,curr_node.y)
            node_2 = (curr_node.parent.x,curr_node.parent.y)
            img = cv2.line(img,node_1,node_2,(0,0,255),2)

            cv2.imshow("img",img)
            cv2.waitKey(1)
        except:
            pass

        return best_cost

    def euclidean_distance(self, node_1, node_2):
        try:
            return np.sqrt( (node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
        except:
            return 0
    
    def sample(self, x_start, x_goal, c_max):
        random_x = random.randint(0,self.width)
        random_y = random.randint(0,self.height)

        random_node = Node(random_x, random_y)

        return random_node

    def informed_sample(self,x_start,x_goal,c_best):
        c_min = self.euclidean_distance(x_start,x_goal)
        x_centre = np.array([(x_start.x + x_goal.x)/2,(x_start.y + x_goal.y)/2,0]).reshape(3,1)

        a1 = np.transpose(np.array([(x_goal.x - x_start.x)/c_min, (x_goal.y - x_start.y)/c_min, 0])).reshape(3,1)
        i1 = np.array([1.0, 0.0, 0.0]).reshape(1,3)
        M = a1 @ i1

        U, Sigma, V_t = np.linalg.svd(M)
        det_U = np.linalg.det(U)
        det_V = np.linalg.det(np.transpose(V_t))
        dim = M.shape[0]
        c_middle_term = np.identity(dim)
        c_middle_term[dim-1,dim-1] = det_U * det_V
        C = U @ c_middle_term @ V_t

        try:
            diag_terms = math.sqrt(c_best**2 - c_min**2)/2
        except:
            diag_terms = c_best/2
            
        L = np.identity(dim) * diag_terms
        L[0,0] = c_best/2

        x_ball_theta = np.random.rand() * 2 * np.pi
        x_ball_rad = np.random.rand()
        x_ball = np.array([x_ball_rad*math.cos(x_ball_theta), x_ball_rad*math.sin(x_ball_theta), 0.0]).reshape(dim,1)

        x_f = C @ L @ x_ball + x_centre

        # print(x_f)
        
        x_rand = Node(int(x_f[0,0]), int(x_f[1,0]))

        # print(x_rand.x,x_rand.y)
        self.major_axis = self.get_best_cost()
        self.minor_axis = diag_terms

        return x_rand
    
    def nearest_node(self, x_rand):
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
        det = self.euclidean_distance(x_nearest, x_rand)

        if(det <= self.step_size):
            return x_rand, cost
        else:
            step_x = int(x_nearest.x + (x_rand.x - x_nearest.x) * self.step_size / det)
            step_y = int(x_nearest.y + (x_rand.y - x_nearest.y) * self.step_size / det)
            
            stepped_node = Node(step_x, step_y)

            cost = self.euclidean_distance(stepped_node, x_nearest)
            
            return stepped_node, cost
    
    def set_node_cost(self,node):
        node_cost = 0.0
        curr_node = node
        if(curr_node.parent is None):
            node_cost = 0.0
        else:
            node_cost = node.parent.cost + self.euclidean_distance(node,node.parent)
        return node_cost
    
    def nodes_slope(self,node_1,node_2):
        num = node_1.y - node_2.y
        den = node_1.x - node_2.x
        return math.atan2(num,den)

    def collision_free(self, x_nearest, x_new, resolution = 0.1):
        if(self.map[x_nearest.x,x_nearest.y,0] == 0 and self.map[x_nearest.x,x_nearest.y,1] == 0 and self.map[x_nearest.x,x_nearest.y,2] == 0):
            return 0
        if(self.map[x_new.x,x_new.y,0] == 0 and self.map[x_new.x,x_new.y,1] == 0 and self.map[x_new.x,x_new.y,2] == 0):
            return 0

        parts = int(1/resolution)
        
        for i in range(1,parts):
            section_x = int((x_nearest.x*i + x_new.x*(parts-i))/parts)
            section_y = int((x_nearest.y*i + x_new.y*(parts-i))/parts)
            if(self.map[section_x,section_y,0] == 0 and self.map[section_x,section_y,1] == 0 and self.map[section_x,section_y,2] == 0):
                return 0
        return 1
    
    def is_valid(self,node):
        if(node.x < self.width and node.y < self.height):
            return True
        return False

    def is_in_obstacle(self,node):
        if(self.map[node.x,node.y,0] ==0 and self.map[node.x,node.y,1] == 0 and self.map[node.x,node.y,2] == 0):
            return 1
        return 0

    def get_nodes_in_radius(self,radius,x_new):
        nodes_in_radius = []
        for node in self.nodes:
            if(self.euclidean_distance(node,x_new) <= radius):
                if(self.collision_free(node,x_new)):
                    nodes_in_radius.append(node)
        return nodes_in_radius

    def delete_edge(self,node_1,node_2):
        for edge in self.edges:
            if((edge.node_1 == node_1 and edge.node_2 == node_2) or (edge.node_1 == node_2 and edge.node_2 == node_1)):
                self.edges.remove(edge)
    
    def delete_all_edges(self,node):
        edges_to_remove = []
        for edge in self.edges:
            if(edge.node_1 == node or edge.node_2 == node):
                edges_to_remove.append(edge)
        # to avoid memory issues
        for edge_ in edges_to_remove:
            self.edges.remove(edge_)

    def rewire(self,x_new,nodes_in_radius):

        rewire_start = time.time()
        
        for node in self.nodes:
            node.cost = self.set_node_cost(node)

        # print("Cost recalc time : ", time.time() - rewire_start)

        cost = np.inf
        best_cost_node = None
        first_sample = False
        continue_rewire = True

        # connect to best-case node
        for node in nodes_in_radius:
            if(node.cost + self.euclidean_distance(node,x_new) < cost):
                best_cost_node = node
                cost = node.cost + self.euclidean_distance(node,x_new)
        
        # print("Nodes in radius time : ", time.time() - rewire_start)
        
        if(best_cost_node is None):
            continue_rewire = False
        
        if(continue_rewire):
            self.delete_all_edges(x_new)
            edge = Edge(best_cost_node,x_new,self.euclidean_distance(best_cost_node,x_new))

            x_new.parent = best_cost_node
            x_new.cost = self.euclidean_distance(best_cost_node,x_new) + best_cost_node.cost
            edge.node_1 = x_new
            edge.node_2 = x_new.parent
            edge.cost = self.euclidean_distance(x_new,x_new.parent)
            if(self.collision_free(best_cost_node,x_new,resolution=0.1)):
                self.edges.append(edge)
            if(len(self.nodes) == 1):
                print("1 node but still rewiring!")
            
            # print("Collision check time : ", time.time() - rewire_start)

            nodes_in_radius.remove(best_cost_node)

            # print("Remove nodes in radius time : ", time.time() - rewire_start)

            # form new edge
            self.nodes.append(x_new)

            parent_edge = Edge(x_new, x_new.parent, self.euclidean_distance(x_new,x_new.parent))

            for node in nodes_in_radius:
                if(self.collision_free(x_new,node)):
                    if(x_new.cost + self.euclidean_distance(x_new,node) < node.cost):
                        if(node in self.x_soln):
                            _ = self.display_informed_converged_map(x_new)
                            pass
                        """
                        if solution is improved, visualize improved solution
                        """
                        node.cost = x_new.cost + self.euclidean_distance(x_new,node)
                        node.parent = x_new
                        self.delete_all_edges(node)
                        edge.node_1 = x_new
                        edge.node_2 = node
                        edge.cost = self.euclidean_distance(x_new,node)
                        self.edges.append(edge)
                    else:
                        pass
                else:
                    # print("Collision while rewiring")
                    continue
                parent_edge.node_1 = node
                parent_edge.node_2 = node.parent
                parent_edge.cost = self.euclidean_distance(node,node.parent)
                self.edges.append(parent_edge)
            # print("Actual rewiring time : ", time.time() - rewire_start)
            best_cost_edge = Edge(x_new,best_cost_node,self.euclidean_distance(x_new,best_cost_node))
            self.edges.append(best_cost_edge)
            # cv2.waitKey(0)

    def get_best_cost(self):
        curr_node = self.goal
        best_cost = 0.0
        while(curr_node.parent is not None):
            best_cost += self.euclidean_distance(curr_node,curr_node.parent)
            curr_node = curr_node.parent
        return best_cost