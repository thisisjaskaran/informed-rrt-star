import random
import json
import informed_rrt_star as irrtstar
import rrt_star as rrtstar
import matplotlib.pyplot as plt
import math

class Logger:
    def __init__(self):
        self.f_json = open('config.json',)
    
        self.data = json.load(self.f_json)
        
        for self.param in self.data['parameters']:
            self.height = self.param['height']
            self.width = self.param['width']
            self.start_pose = self.param['start_pose']
            self.goal_pose = self.param['goal_pose']
            self.step_size = self.param['step_size']
            self.search_radius = self.param['search_radius']
            self.ITERATIONS = self.param['ITERATIONS']
            self.show_edges = self.param['show_edges']
            self.show_sample = self.param['show_sample']
            self.show_ellipse = self.param['show_ellipse']
            self.threshold_cost = self.param['threshold_cost']
            self.generate_random_map = self.param['generate_random_map']
            self.num_random_obstacles = self.param['num_random_obstacles']
            self.rand_obstacle_size = self.param['rand_obstacle_size']
            self.min_obstacle_size = self.param['min_obstacle_size']
            self.num_tests = self.param['num_tests']
            self.show_plot = self.param['show_plot']
    
        self.f_json.close()

        self.rand_xs = []
        self.rand_ys = []
        self.rand_widths = []
        self.rand_heights = []

        for i in range(self.num_random_obstacles):
            self.rand_xs.append(random.randint(self.min_obstacle_size,self.width - self.rand_obstacle_size))
            self.rand_ys.append(random.randint(self.min_obstacle_size,self.height - self.rand_obstacle_size))
            self.rand_widths.append(random.randint(self.min_obstacle_size,self.rand_obstacle_size))
            self.rand_heights.append(random.randint(self.min_obstacle_size,self.rand_obstacle_size))

    def test_informed_rrt_star(self):
        total_time, total_cost = irrtstar.do_informed_rrt_star(self.rand_xs,self.rand_ys,self.rand_widths,self.rand_heights)
        return total_time, total_cost
    
    def test_rrt_star(self):
        total_time, total_cost = rrtstar.do_rrt_star(self.rand_xs,self.rand_ys,self.rand_widths,self.rand_heights)
        return total_time, total_cost

    def refreshed_randomised(self):
        self.rand_xs = []
        self.rand_ys = []
        self.rand_widths = []
        self.rand_heights = []

        for i in range(self.num_random_obstacles):
            self.rand_xs.append(random.randint(self.min_obstacle_size,self.width - self.rand_obstacle_size))
            self.rand_ys.append(random.randint(self.min_obstacle_size,self.height - self.rand_obstacle_size))
            self.rand_widths.append(random.randint(self.min_obstacle_size,self.rand_obstacle_size))
            self.rand_heights.append(random.randint(self.min_obstacle_size,self.rand_obstacle_size))
    
    def plot_test_times(self,test_times_1,test_times_2,num_iters,label_1,label_2):
        fig = plt.figure()
        fig.suptitle('Run times', fontsize=15)

        axes = plt.gca()
        axes.set_xlim([1,self.num_tests])
        axes.set_ylim([0,max(max(test_times_1),max(test_times_2))+20])

        plt.plot(num_iters,test_times_1)
        plt.plot(num_iters,test_times_2,label=label_2)
        plt.legend([label_1,label_2])
        fig.savefig('plots/times.png')
        plt.show()
    
    def plot_test_costs(self,test_costs_1,test_costs_2,num_iters,label_1,label_2):
        fig = plt.figure()
        fig.suptitle('Path costs', fontsize=15)

        axes = plt.gca()
        axes.set_xlim([1,self.num_tests])
        axes.set_ylim([0,max(max(test_costs_1),max(test_costs_2))+20])

        plt.plot(num_iters,test_costs_1)
        plt.plot(num_iters,test_costs_2)
        plt.legend([label_1,label_2])
        fig.savefig('plots/costs.png')
        plt.show()