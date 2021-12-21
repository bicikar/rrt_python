import sys
import time
from statistics import mean
from field import *

if __name__ == '__main__':
    n = len(sys.argv)
    if n < 3:
        raise ValueError('You should input at least map name and iterations')
    map_name = sys.argv[1]
    iterations = int(sys.argv[2])
    step = int(sys.argv[3]) if n >= 4 else 0
    star = int(sys.argv[4]) if n >= 5 else 0

    field_origin = Field('example_map0', 100, 0)
    iterations = (500, 1000, 2000, 5000, 7000, 10000)
    mean_times = []
    mean_costs = []
    for iter in iterations:
        time_results = []
        cost_results = []
        for i in range(100):
            start = time.time()
            field_origin.rrt_algo(iter)
            end = time.time()
            time_results.append(end - start)
            cost_results.append(field_origin._result_node.cost)
        mean_times.append(mean(time_results))
        mean_costs.append(mean(cost_results))
        print(iter)
    print('Iterations:', iterations)
    print('Mean times:', mean_times)
    print('Mean costs:', mean_costs)


    # field.rrt_algo(iterations)
    # print('Final cost:',field._result_node.cost)
    # print('Result node:', field._result_node)
    #
    # field.draw_all()
    # field.draw()
