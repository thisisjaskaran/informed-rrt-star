import cv2
import numpy as np
import random
import utils

class Map():

    def __init__(self, height, width, step_size):
        self.height = height
        self.width = width
        self.step_size = step_size
        self.map =  np.ones((self.height,self.width,3), np.uint8) * 255

        self.nodes = [] # list of tuples, where each tuple is a coordinate
        self.edges = [] # list of tuples, where each tuple is a set of indices of corressponding nodes
    
    def add_start(self,x,y):
        self.map[x,y] = (0,255,0)
        self.add_node(x,y)
    
    def add_goal(self,x,y):
        self.map[x,y] = (0,0,255)
    
    def add_obstacle(self, x, y):
        self.map[x,y] = (0,0,0)
    
    def display_map(self, display_tree = True):
        img = self.map
        if(display_tree):
            for edge in self.edges:
                # print(self.nodes[edge[0]],self.nodes[edge[1]])
                img = cv2.line(img,self.nodes[edge[0]],self.nodes[edge[1]],(255,0,0),2)
        # print("tring to display")
        cv2.imshow("map", img)
        cv2.waitKey(10)
    
    def sample_point(self):

        random_x = random.randint(0,self.height)
        random_y = random.randint(0,self.width)

        while(self.map[random_x,random_y,0] == 0 and self.map[random_x,random_y,1] == 0 and self.map[random_x,random_y,2] == 0):
            random_x = random.randint(0,self.height)
            random_y = random.randint(0,self.width)    
        
        return random_x,random_y
    
    def find_nearest(self,x,y):

        min_dist = np.inf
        closest_node = None

        found_nearest_node = False

        # check list of nodes to find closest node
        for node in self.nodes:
            point = [x,y]

            det = np.sqrt((x-node[0])**2 + (y-node[1])**2)
            step_to_sample_x = int((x-node[0]) * self.step_size / det + node[0])
            step_to_sample_y = int((y-node[1]) * self.step_size / det + node[1])

            # if no collision, find closest node to sampled point
            if( utils.euclidean_distance(point,list(node)) < min_dist\
                and self.map[step_to_sample_x,step_to_sample_y,0] == 255\
                and self.map[step_to_sample_x,step_to_sample_y,1] == 255\
                and self.map[step_to_sample_x,step_to_sample_y,2] == 255):

                min_dist = utils.euclidean_distance(point,node)
                closest_node = node
                found_nearest_node = True

        return found_nearest_node, closest_node

    def add_node(self,x,y):
        self.nodes.append((x,y))
    
    def add_edge(self,node_1,node_2):
        self.edges.append((node_1,node_2))
    
    def take_step(self,nearest_node,random_point):
        norm = utils.euclidean_distance(nearest_node,random_point)
        if(norm > self.step_size):
            scale = self.step_size/norm
            step_vector = [int(scale * (random_point[0] - nearest_node[0]) + nearest_node[0]) , int(scale * (random_point[1] - nearest_node[1]) + nearest_node[1]) ]
            return step_vector
        else:
            return random_point

if __name__=="__main__":
    map = Map(500,800)

    point_1 = [0,0]
    point_2 = [20,10]

    points_in_line = map.find_points_in_line(point_1,point_2)

    print(points_in_line)