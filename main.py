"""Read arguments and run rrt algorithm."""
import sys

from field import Field

if __name__ == '__main__':
    argc = len(sys.argv)
    if argc < 3:
        raise ValueError('You should input at least map name and iterations')
    map_name = sys.argv[1]
    iterations = int(sys.argv[2])
    step = int(sys.argv[3]) if argc >= 4 else 0
    star = int(sys.argv[4]) if argc >= 5 else 1
    gif = int(sys.argv[5]) if argc >= 6 else 0
    field = Field(map_name, step, star, gif)

    field.rrt_algo(iterations)

    field.save_pic()
