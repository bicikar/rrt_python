import sys
import time
from statistics import mean
from field import *

if __name__ == '__main__':
    random.seed(1234)
    n = len(sys.argv)
    if n < 3:
        raise ValueError('You should input at least map name and iterations')
    map_name = sys.argv[1]
    iterations = int(sys.argv[2])
    step = int(sys.argv[3]) if n >= 4 else 0
    star = int(sys.argv[4]) if n >= 5 else 0

    field = Field(map_name, step, star)


    field.rrt_algo(iterations)

    field.draw_all()
    field.draw()
