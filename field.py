"""Main field and algorithm class."""

import random

from PIL import Image, ImageDraw

from kdtree import distance, KDTree, Node, Point, Tree
from obstacles import Polygon


class Field(object):
    """Field with rrt algorithm."""

    def __init__(self, file_name, step=0, star=1, gif=0):
        """Init the field."""
        self._step = step
        self._obstacles = []
        self._path_vector = []
        self._result_node = None
        file_name = '{0}{1}'.format('maps/', file_name)
        with open(file_name, 'r') as map_file:
            lines = map_file.readlines()
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
            self.step_scale = 20
            self.radius_scale = 1.5
            self.gif_flag = gif

            self._tree = Tree(self._start)
            if self._step <= 0:
                self._step = min(self._rows, self._cols) // self.step_scale

            self._kdtree = KDTree(self._start)

            for line in lines[4:]:
                points = list(map(float, line.split()))
                self._obstacles.append(Polygon())
                for ind in range(0, len(points) // 2):
                    self._obstacles[-1].add_point(
                        Point(points[2 * ind], points[2 * ind + 1]),
                    )

    def point_obstacles(self, node):
        """Return true if point intersects obstacle."""
        for obstacle in self._obstacles:
            if obstacle.check_inside(node):
                return True
        return False

    def segment_obstacles(self, start, end):
        """Return true if segment intersects obstacle."""
        for obstacle in self._obstacles:
            if obstacle.segment_obstacle(start, end):
                return True
        return False

    def take_random_point(self):
        """Return random point in free space."""
        cor_x = random.uniform(0, self._cols)
        cor_y = random.uniform(0, self._rows)
        random_node = Node(cor_x, cor_y)
        if not self.point_obstacles(random_node):
            return random_node
        return None

    def find_nearest(self, node):
        """Find the nearest point to current node."""
        return self._kdtree.nearest(node)

    def find_nearest_in_radius(self, node):
        """Find all points within the point radius."""
        nodes = []
        if self.star:
            nodes = self._kdtree.radius_search(
                node,
                self.radius_scale * self._step,
            )

        return nodes

    def steer(self, rand_point, nearest_node):
        """Steer to the random point from the nearest node."""
        dx = rand_point.cor_x - nearest_node.cor_x
        dy = rand_point.cor_y - nearest_node.cor_y
        vec_length = (dx ** 2 + dy ** 2) ** 0.5
        vecx = dx / vec_length * self._step
        vecy = dy / vec_length * self._step
        if not (
            nearest_node.cor_x + vecx < 0
            or nearest_node.cor_x + vecx > self._cols
            or nearest_node.cor_y + vecy < 0
            or nearest_node.cor_y + vecy > self._rows
        ):
            dis = distance(
                rand_point.cor_x,
                rand_point.cor_y,
                nearest_node.cor_x,
                nearest_node.cor_y,
            )
            if dis < self._step:
                steer_point = rand_point
            else:
                steer_point = Node(
                    nearest_node.cor_x + vecx,
                    nearest_node.cor_y + vecy,
                )
            if self.segment_obstacles(nearest_node, steer_point):
                return None

            steer_point.cost = nearest_node.cost + dis
            return steer_point

    def choose_parent(self, node, near_nodes, nearest_node):
        """Choose parent to minimize total cost."""
        min_node = nearest_node
        min_cost = nearest_node.cost + distance(
            node.cor_x,
            node.cor_y,
            nearest_node.cor_x,
            nearest_node.cor_y,
        )
        for par in near_nodes:
            if not self.segment_obstacles(node, par):
                dis = distance(node.cor_x, node.cor_y, par.cor_x, par.cor_y)
                cur_cost = par.cost + dis
                if cur_cost < min_cost:
                    min_cost = cur_cost
                    min_node = par
        return min_node

    def rewire(self, node, near_nodes):
        """Rewire existing points to minimize cost."""
        for par in near_nodes:
            if not self.segment_obstacles(node, par):
                dis = distance(node.cor_x, node.cor_y, par.cor_x, par.cor_y)
                if node.cost + dis < par.cost:
                    par.parent.delete_child(par)
                    par.delete_child(node)
                    par.define_parent(node)
                    node.add_child(par)
                    par.cost = node.cost + dis

    def rrt_algo(self, iterations):
        """Run rrt algorithm."""
        for _ in range(iterations):
            random_point = self.take_random_point()
            while random_point is None:
                random_point = self.take_random_point()

            nearest_node = self.find_nearest(random_point)
            steer_point = self.steer(random_point, nearest_node)
            if steer_point is not None:
                if self.gif_flag:
                    self.draw_iter(steer_point)
                near_nodes = self.find_nearest_in_radius(steer_point)

                parent = self.choose_parent(
                    steer_point,
                    near_nodes,
                    nearest_node,
                )
                parent.add_child(steer_point)
                steer_point.define_parent(parent)
                self.max_depth = max(self.max_depth, steer_point.depth)
                self._kdtree.insert(self._kdtree.parent, steer_point)
                self.rewire(steer_point, near_nodes)

                if self.gif_flag:
                    self.draw_iter()
                if (
                    distance(
                        self._end.cor_x,
                        self._end.cor_y,
                        steer_point.cor_x,
                        steer_point.cor_y,
                    ) <= self._step
                ):
                    if not self.segment_obstacles(steer_point, self._end):
                        if self._result_node is None:
                            self._result_node = steer_point
                            self._end.parent = self._result_node
                        elif self._result_node.cost > steer_point.cost:
                            dis_result = self._result_node.cost + distance(
                                self._end.cor_x,
                                self._end.cor_y,
                                self._result_node.cor_x,
                                self._result_node.cor_y,
                            )
                            dis_steer = steer_point.cost + distance(
                                self._end.cor_x,
                                self._end.cor_y,
                                steer_point.cor_x,
                                steer_point.cor_y,
                            )
                            if dis_steer < dis_result:
                                self._result_node = steer_point
                                self._end.parent = self._result_node
        self.draw_iter()

    def draw_tree(self, node, draw, deep):
        """Draw tree."""
        if deep >= 0:
            small_rad_scale = 200
            if not node.child:
                return
            for child in node.child:
                draw.line(
                    [(node.cor_x, node.cor_y), (child.cor_x, child.cor_y)],
                    fill=(31, 31, 31),
                    width=3,
                )
                rad = min(self._rows, self._cols) / small_rad_scale
                draw.ellipse(
                    (
                        child.cor_x - rad,
                        child.cor_y - rad,
                        child.cor_x + rad,
                        child.cor_y + rad,
                    ),
                    fill=(31, 31, 31),
                )
                self.draw_tree(child, draw, deep - 1)

    def draw_iter(self, cur_point=None):
        """Draw current state of the field."""
        rad_scale = 125
        big_rad_scale = 1.25
        image = Image.new('RGB', (self._cols, self._rows), (255, 255, 255))
        draw = ImageDraw.Draw(image)
        for obstacle in self._obstacles:
            obs_points = [
                (point.cor_x, point.cor_y) for point in obstacle.points
            ]
            draw.polygon(obs_points, fill='gray')
        self.draw_tree(self._tree.root, draw, self.max_depth + 1)
        rad = min(self._rows, self._cols) / rad_scale
        big_r = rad * big_rad_scale
        draw.ellipse(
            (
                self._start.cor_x - big_r,
                self._start.cor_y - big_r,
                self._start.cor_x + big_r,
                self._start.cor_y + big_r,
            ), fill='red',
        )
        draw.ellipse(
            (
                self._end.cor_x - big_r,
                self._end.cor_y - big_r,
                self._end.cor_x + big_r,
                self._end.cor_y + big_r,
            ), fill='blue',
        )
        self.read_parent(self._end, draw)
        if cur_point is not None:
            draw.ellipse(
                (
                    cur_point.cor_x - big_r,
                    cur_point.cor_y - big_r,
                    cur_point.cor_x + big_r,
                    cur_point.cor_y + big_r,
                ), fill=(22, 232, 19),
            )
        self.image_list.append(image)

    def save_pic(self):
        """Save picture and gif."""
        self.image_list[-1].save('res.png')
        if self.gif_flag:
            self.image_list[0].save(
                'res.gif',
                save_all=True,
                append_images=self.image_list[1:],
                optimize=False,
                duration=150,
                loop=0,
            )

    def read_parent(self, node, draw):
        """Draw shortest path from end."""
        rad_scale = 125
        rad = min(self._rows, self._cols) / rad_scale
        self._path_vector.append(node)
        parent = node.parent
        if node.parent is not None:
            draw.line(
                [(node.cor_x, node.cor_y), (parent.cor_x, parent.cor_y)],
                fill='red',
                width=11,
            )
            draw.ellipse(
                (
                    node.cor_x - rad,
                    node.cor_y - rad,
                    node.cor_x + rad,
                    node.cor_y + rad,
                ), fill=(30, 170, 10),
            )
            self.image_iter += 1
            self.read_parent(parent, draw)
