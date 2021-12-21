import random

from PIL import Image, ImageDraw

from kdtree import *
from obstacles import *


class Field:
    def __init__(self, file_name, step=0, star=0):
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
        self.max_depth = 0
        self.star = star

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
        for obstacle in self._obstacles:
            if obstacle.check_inside(node):
                return True
        return False

    def segment_obstacles(self, start, end):
        for obstacle in self._obstacles:
            if obstacle.segment_obstacle(start, end):
                return True
        return False

    def take_random_point(self):
        x = random.uniform(0, self._cols)
        y = random.uniform(0, self._rows)
        random_node = Node(x, y)
        if not self.point_obstacles(random_node):
            return random_node
        return None

    def find_nearest(self, node):
        min_node = self._kdtree.nearest(node)
        return min_node

    def find_nearest_in_radius(self, node):
        nodes = []
        if self.star:
            nodes = self._kdtree.radius_search(node, 1.5 * self._step)
        return nodes
        # res = []
        # if self.star:
        #     for p in self._tree._nodes:
        #         if distance(node._x, node._y, p._x,
        #                     p._y) < 1.5 * self._step:
        #             res.append(p)
        # return res

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
        dis = distance(rand_point._x, rand_point._y, nearest_node._x,
                       nearest_node._y)
        dis_squared = distance(rand_point._x, rand_point._y, nearest_node._x,
                               nearest_node._y)
        if dis < self._step:
            steer_point = rand_point
        else:
            steer_point = Node(nearest_node._x + vec_x, nearest_node._y + vec_y)
        if self.segment_obstacles(nearest_node, steer_point):
            return None

        steer_point.cost = nearest_node.cost + dis
        return steer_point

    def choose_parent(self, node, near_nodes, nearest_node):
        min_node = nearest_node
        min_cost = nearest_node.cost + distance(node._x, node._y,
                                                nearest_node._x,
                                                nearest_node._y)
        for par in near_nodes:
            if not self.segment_obstacles(node, par):
                dis = distance(node._x, node._y, par._x, par._y)
                cur_cost = par.cost + dis
                if cur_cost < min_cost:
                    min_cost = cur_cost
                    min_node = par
        return min_node

    def rewire(self, node, near_nodes):
        for p in near_nodes:
            if not self.segment_obstacles(node, p):
                dis = distance(node._x, node._y, p._x, p._y)
                if node.cost + dis < p.cost:
                    p.parent.delete_child(p)
                    p.delete_child(node)
                    p.define_parent(node)
                    node.add_child(p)
                    p.cost = node.cost + dis

    def rrt_algo(self, iterations):
        for i in range(iterations):
            random_point = self.take_random_point()
            while random_point is None:
                random_point = self.take_random_point()

            nearest_node = self.find_nearest(random_point)
            steer_point = self.steer(random_point, nearest_node)
            if steer_point is not None:
                # self.draw_iter(steer_point)
                near_nodes = self.find_nearest_in_radius(steer_point)

                parent = self.choose_parent(steer_point, near_nodes,
                                            nearest_node)
                parent.add_child(steer_point)
                steer_point.define_parent(parent)
                self.max_depth = max(self.max_depth, steer_point.depth)
                self._tree.add_node(steer_point)
                self._kdtree.insert(self._kdtree.parent, steer_point)
                self.rewire(steer_point, near_nodes)

                # self.draw_iter()
                if (distance(self._end._x, self._end._y, steer_point._x,
                             steer_point._y) < self._step):
                    if not self.segment_obstacles(steer_point, self._end):
                        if self._result_node is None:
                            self._result_node = steer_point
                            self._end.parent = self._result_node
                        else:
                            if self._result_node.cost > steer_point.cost:
                                dis_result = self._result_node.cost + distance(
                                    self._end._x,
                                    self._end._y,
                                    self._result_node._x,
                                    self._result_node._y
                                )
                                dis_steer = steer_point.cost + distance(
                                    self._end._x,
                                    self._end._y,
                                    steer_point._x,
                                    steer_point._y
                                )
                                if dis_steer < dis_result:
                                    self._result_node = steer_point
                                    self._end.parent = self._result_node

                            # if self._result_node is not None:
            #     self._end.parent = self._result_node
            #     break

    def draw_tree(self, node, draw, i):
        if i >= 0:
            if len(node._child) == 0:
                return
            for child in node._child:
                draw.line([(node._x, node._y), (child._x, child._y)],
                          fill='black',
                          width=4)
                rad = min(self._rows, self._cols) / 200
                draw.ellipse((child._x - rad, child._y - rad,
                              child._x + rad,
                              child._y + rad),
                             fill='black')
                self.draw_tree(child, draw, i - 1)

    def draw_iter(self, cur_point=None):
        image = Image.new("RGB", (self._cols, self._rows), (255, 255, 255))
        draw = ImageDraw.Draw(image)
        for obstacle in self._obstacles:
            obs_points = [(point._x, point._y) for point in obstacle.points]
            draw.polygon(obs_points, fill='gray')
        self.draw_tree(self._tree.root, draw, self.max_depth + 1)
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
        if cur_point is not None:
            draw.ellipse((cur_point._x - big_r, cur_point._y - big_r,
                          cur_point._x + big_r,
                          cur_point._y + big_r),
                         fill=(22, 232, 19))
        self.image_list.append(image)

    def draw_all(self):
        # for i in range(self.max_depth + 1):
        #     self.draw_iter(i)
        image = Image.new("RGB", (self._cols, self._rows), (255, 255, 255))
        draw = ImageDraw.Draw(image)
        for obstacle in self._obstacles:
            obs_points = [(point._x, point._y) for point in obstacle.points]
            draw.polygon(obs_points, fill='gray')
        self.draw_tree(self._tree.root, draw, self.max_depth + 1)
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
        for i in range(10):
            self.image_list.append(image)
        self.image_list[0].save('res.gif', save_all=True,
                                append_images=self.image_list[1:],
                                optimize=False, duration=350, loop=0)

    def draw(self):
        image = Image.new("RGB", (self._cols, self._rows), (255, 255, 255))
        draw = ImageDraw.Draw(image)
        for obstacle in self._obstacles:
            obs_points = [(point._x, point._y) for point in obstacle.points]
            draw.polygon(obs_points, fill='gray')
        self.draw_tree(self._tree.root, draw, self.max_depth)
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
        image.save('res.png')

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
