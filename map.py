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
        # self.free_points = []

        # self.list_free()

        self.nodes = [] # list of tuples, where each tuple is a coordinate
        self.edges = [] # list of tuples, where each tuple is a set of indices of corressponding nodes
    
    def add_start(self,x,y):
        self.map[x,y] = (0,255,0)
        # self.free_points.remove([x,y])
        self.add_node(x,y)
    
    def add_goal(self,x,y):
        self.map[x,y] = (0,0,255)
        # self.free_points.remove([x,y])
    
    def add_obstacle(self, x, y):
        self.map[x,y] = (0,0,0)
        # self.free_points.remove([x,y])
    
    def display_map(self, display_tree = True):
        img = self.map
        if(display_tree):
            for edge in self.edges:
                # print(self.nodes[edge[0]],self.nodes[edge[1]])
                img = cv2.line(img,self.nodes[edge[0]],self.nodes[edge[1]],(255,0,0),2)
        cv2.imshow("map", img)
        cv2.waitKey(5)

    # def list_free(self):
    #     for i in range(self.height):
    #         for j in range(self.width):
    #             if(self.map[i,j,0] == 255 and self.map[i,j,1] == 255 and self.map[i,j,2] == 255):
    #                 self.free_points.append([i,j])
    
    def sample_point(self):
        # num_free_points = len(self.free_points)
        # random_point = random.randint(0,num_free_points)
        # point = self.free_points[random_point] 
        random_x = random.randint(0,self.height)
        random_y = random.randint(0,self.width)

        while(self.map[random_x,random_y,0] == 0 and self.map[random_x,random_y,1] == 0 and self.map[random_x,random_y,2] == 0):
            random_x = random.randint(0,self.width)
            random_y = random.randint(0,self.height)    
        
        return random_x,random_y
    
    def find_nearest(self,x,y):
    
        """
        There is no need to check if an obstacle lies in the way
        All you need to check is if the new node is in an obstacle
        """
        min_dist = np.inf
        closest_node = None

        no_nearest_node = True

        # check list of nodes to find closest node
        for node in self.nodes:
            point = [x,y]
            # print("Checking node ",node)
            # points_in_line = self.find_points_in_line(point,list(node))
            # collision = False

            # # make sure connecting line does not have collisions
            # for line_point in points_in_line:
            #     if(self.map[line_point[0],line_point[1],0] == 0 and self.map[line_point[0],line_point[1],1] == 0 and self.map[line_point[0],line_point[1],2] == 0):
            #         # print("Collision found. Move to next node.")
            #         collision = True
            #         break
            # if(collision == True):
            #     continue

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
                no_nearest_node = False


        return no_nearest_node, closest_node

    # def find_points_in_line(self, point_1, point_2):
    #     temp_img_for_line = np.ones((self.height,self.width,3), np.uint8) * 255
    #     temp_img_for_line = cv2.line(temp_img_for_line,tuple(point_1),tuple(point_2),(0,0,0),1)

    #     # cv2.imshow("line",temp_img_for_line)
    #     # cv2.waitKey(0)

    #     points_in_line = []

    #     for i in range(self.height):
    #         for j in range(self.width):
    #             if(temp_img_for_line[i,j,0] == 0):
    #                 points_in_line.append((j,i))
        
        return points_in_line

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