import sys

from field import *

if __name__ == '__main__':
    random.seed(299124)
    n = len(sys.argv)
    if n < 3:
        raise ValueError('You should input at least map name and iterations')
    map_name = sys.argv[1]
    iterations = int(sys.argv[2])
    print('args:', n)
    print(sys.argv)
    step = int(sys.argv[3]) if n >= 4 else 0
    print("read_step", step)
    star = int(sys.argv[4]) if n >= 5 else 0
    field = Field(map_name, step, star)

    field.rrt_algo(iterations)
    print(field.max_depth)
    print('step:', field._step)

    print("a")
    # print(field._start)
    # print(*field._start._child)
    # field._start.delete_child(Node(130.95263291479375, 93.5293911779537))
    # print("New child:", *field._start._child)

    field.draw()

    # for i in range(15):
    #     field.image_list.append(field.image_list[-1])
    # field.image_list[0].save('res.gif', save_all=True,
    #                         append_images=field.image_list[1:], duration=150,
    #                         loop=1)
