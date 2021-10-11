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

    with open('results.csv', mode='w') as output_file:
        output_logger = csv.writer(output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        output_logger.writerow(["Map", "RRT* Time", "I-RRT* Time", "RRT* Cost", "I-RRT* Cost"])

        for i in range(num_tests):
            map_no = i + 1

            test_iters.append(i+1)
            logger.refreshed_randomised()

            test_time, test_cost = logger.test_informed_rrt_star()
            informed_times.append(test_time)
            informed_costs.append(test_cost)

            test_time, test_cost = logger.test_rrt_star()
            rrt_star_times.append(test_time)
            rrt_star_costs.append(test_cost)

            print(f"Map : {map_no}")
            print(f"RRT* Time : {rrt_star_times[i]} RRT* Cost : {rrt_star_costs[i]}")
            print(f"I-RRT* Time : {informed_times[i]} I-RRT* Cost : {informed_costs[i]}")

            output_logger.writerow([i+1,rrt_star_times[i],informed_times[i],rrt_star_costs[i],informed_costs[i]])
    
    if(logger.show_plot):
        # logger.plot_test_times(informed_times,rrt_star_times,test_iters,label_1="I-RRT*",label_2="RRT*")
        logger.plot_test_costs(informed_costs,rrt_star_costs,test_iters,label_1="I-RRT*",label_2="RRT*")
    