import sys

from field import *

if __name__ == '__main__':
    n = len(sys.argv)
    if n < 3:
        raise ValueError('You should input at least map name and iterations')
    map_name = sys.argv[1]
    iterations = int(sys.argv[2])
    step = int(sys.argv[3]) if n == 4 else 0
    field = Field(map_name, step)

    field.rrt_algo(iterations)
    field.draw()
