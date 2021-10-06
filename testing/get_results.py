import logger as lg
import csv

if __name__ == "__main__":
    logger = lg.Logger()
    
    num_tests = logger.num_tests

    test_iters = []

    informed_times = []
    informed_costs = []

    rrt_star_times = []
    rrt_star_costs = []

    # with open('results.csv', mode='w') as output_file:
    #     output_logger = csv.writer(output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #     output_logger.writerow(["Map", "Informed RRT* Time", "Informed RRT* Cost", "RRT* Time", "RRT* Cost"])

    for i in range(num_tests):
        test_iters.append(i+1)
        logger.refreshed_randomised()

        test_time, test_cost = logger.test_informed_rrt_star()
        informed_times.append(test_time)
        informed_costs.append(test_cost)

        test_time, test_cost = logger.test_rrt_star()
        rrt_star_times.append(test_time)
        rrt_star_costs.append(test_cost)

        print(f"Iter : {i} Test Time : {test_time} Test Cost : {test_cost}")

            # output_logger.writerow([i+1,informed_times[i],informed_costs[i],rrt_star_times[i],rrt_star_costs[i]])
    
    # logger.plot_test_times(informed_times,rrt_star_times,test_iters,label_1="informed rrt*",label_2="rrt*")
    logger.plot_test_costs(informed_costs,rrt_star_costs,test_iters,label_1="informed rrt*",label_2="rrt*")
    