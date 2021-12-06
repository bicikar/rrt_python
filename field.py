import random

from PIL import Image, ImageDraw

from kdtree import *
from obstacles import *


class Field:
    def __init__(self, file_name, step=0):
        self._step = step
        self._obstacles = []
        self._path_vector = []
        self._result_node = None
        file_name = 'maps/' + file_name
        file = open(file_name, 'r')
        lines = file.readlines()
        self._cols = int(lines[0])
        self._rows = int(lines[1])
        start = list(map(float, lines[2].split()))
        self._start = Node(start[0], start[1])
        end = list(map(float, lines[3].split()))
        self._end = Node(end[0], end[1])
        self.image = None
        self.image_list = []
        self.image_max_iter = 1
        self.image_iter = 0

        self._tree = Tree(self._start)
        if self._step <= 0:
            self._step = min(self._rows, self._cols) // 20

        self._kdtree = KDTree(self._start)

        for line in lines[4:]:
            points = list(map(float, line.split()))
            self._obstacles.append(Polygon())
            for i in range(0, len(points) // 2):
                self._obstacles[-1].add_point(
                    Point(points[2 * i], points[2 * i + 1])
                )
        file.close()

    def point_obstacles(self, node):
        result = True
        for obstacle in self._obstacles:
            if not obstacle.check_inside(node):
                result = False
        return result

    def take_random_point(self):
        x = random.uniform(0, self._cols)
        y = random.uniform(0, self._rows)
        random_node = Node(x, y)
        if self.point_obstacles(random_node):
            return random_node
        return None

    def find_nearest(self, node):
        min_node = self._kdtree.nearest(node)
        return min_node

    def steer(self, rand_point, nearest_node):
        dx = rand_point._x - nearest_node._x
        dy = rand_point._y - nearest_node._y
        vec_length = (dx ** 2 + dy ** 2) ** 0.5
        vec_x = dx / vec_length * self._step
        vec_y = dy / vec_length * self._step
        if nearest_node._x + vec_x < 0 or \
                nearest_node._x + vec_x > self._cols or \
                nearest_node._y + vec_y < 0 or \
                nearest_node._y + vec_y > self._rows:
            return
        if (distance(rand_point._x, rand_point._y, nearest_node._x,
                     nearest_node._y) ** 0.5 < self._step):
            steer_point = rand_point
        else:
            steer_point = Node(nearest_node._x + vec_x, nearest_node._y + vec_y)
        for obstacle in self._obstacles:
            if not obstacle.segment_obstacle(nearest_node, steer_point):
                return
        nearest_node.add_child(steer_point)
        steer_point.parent = nearest_node
        self._tree.add_node(steer_point)
        self._kdtree.insert(self._kdtree.parent, steer_point)
        if (distance(self._end._x, self._end._y, steer_point._x,
                     steer_point._y) ** 0.5 < self._step):
            self._result_node = steer_point
            print(steer_point._x, steer_point._y)

    def rrt_algo(self, iterations):
        for i in range(iterations):
            random_point = self.take_random_point()
            while random_point is None:
                random_point = self.take_random_point()

            nearest_node = self.find_nearest(random_point)
            self.steer(random_point, nearest_node)
            if self._result_node is not None:
                self._end.parent = self._result_node
                break

    def draw_tree(self, node, draw):
        if len(node._child) > 0:
            for child in node._child:
                draw.line([(node._x, node._y), (child._x, child._y)], fill='black',
                          width=4)
                self.image_iter += 1
                self.draw_tree(child, draw)

    def draw(self):
        self.image = Image.new("RGB", (self._cols, self._rows), (255, 255, 255))
        draw = ImageDraw.Draw(self.image)
        for obstacle in self._obstacles:
            obs_points = [(point._x, point._y) for point in obstacle.points]
            draw.polygon(obs_points, fill='gray')
        self.draw_tree(self._tree.root, draw)
        self.read_parent(self._end, draw)
        rad = min(self._rows, self._cols) / 125
        big_r = rad * 1.25
        draw.ellipse((self._start._x - big_r, self._start._y - big_r,
                      self._start._x + big_r,
                      self._start._y + big_r),
                     fill='red')
        draw.ellipse((self._end._x - big_r, self._end._y - big_r,
                      self._end._x + big_r,
                      self._end._y + big_r),
                     fill='blue')
        self.image.save('res.png')

    def read_parent(self, node, draw):
        rad = min(self._rows, self._cols) / 125
        self._path_vector.append(node)
        parent = node.parent
        if node.parent is not None:
            draw.line([(node._x, node._y), (parent._x, parent._y)], fill='red',
                      width=5)
            draw.ellipse(
                (node._x - rad, node._y - rad, node._x + rad, node._y + rad),
                fill=(30, 170, 10))
            self.image_iter += 1
            self.read_parent(parent, draw)
