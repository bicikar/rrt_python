class Point(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def get(self, index):
        if index == 0:
            return self._x
        if index == 1:
            return self._y


class Node(object):
    def __init__(self, x=0, y=0):
        self._x = x
        self._y = y
        self._child = []
        self.parent = None
        self.left = None
        self.right = None
        self._same = []
        self.cost = 0
        self.depth = 0

    def setx(self, x):
        self._x = x

    def sety(self, y):
        self._y = y

    def add_child(self, node):
        self._child.append(node)

    def add_same(self, node):
        self._same.append(node)

    def define_parent(self, parent):
        self.parent = parent
        self.depth = parent.depth + 1
        dis = distance(self._x, self._y, parent._x, parent._y)
        self.cost = parent.cost + dis

    def get(self, index):
        if index == 0:
            return self._x
        else:
            return self._y

    def delete_child(self, node):
        # new_child = [e for e in self._child if e._x != node._x or e._y != node._y]
        # self._child = new_child
        for i, child in enumerate(self._child):
            if abs(child._x - node._x) < 0.0001 and abs(child._y - node._y) < 0.0001:
                self._child[i].parent = None
                del self._child[i]

    def __str__(self):
        return "({0} {1})".format(self._x, self._y)


class KDTree:
    def __init__(self, parent):
        self.parent = parent
        self._best = None
        self._best_dist = 0
        self._dim = 2
        self._nodes_in_radius = []

    def insert_rec(self, point, node, s_dim):
        if node is None:
            return point
        if point.get(s_dim) < node.get(s_dim):
            node.left = self.insert_rec(
                point,
                node.left,
                (s_dim + 1) % self._dim,
            )
        elif point.get(s_dim) > node.get(s_dim):
            node.right = self.insert_rec(
                point,
                node.right,
                (s_dim + 1) % self._dim,
            )
        else:
            node.add_same(point)
        return node

    def insert(self, root, point):
        return self.insert_rec(point, root, 0)

    def nearest_rec(self, root, point, index):
        if root is None:
            return
        dis = distance(root._x, root._y, point._x, point._y)
        if self._best is None or dis < self._best_dist:
            self._best_dist = dis
            self._best = root
        if self._best_dist == 0:
            return
        dx = root.get(index) - point.get(index)
        index = (index + 1) % self._dim
        self.nearest_rec(root.left if dx > 0 else root.right, point, index)
        if dx ** 2 >= self._best_dist ** 2:
            return
        self.nearest_rec(root.right if dx > 0 else root.left, point, index)

    def nearest(self, point):
        if self.parent is None:
            return
        self._best = None
        self._best_dist = 0
        self.nearest_rec(self.parent, point, 0)
        return self._best

    def radius_rec(self, root, point, index, radius):
        if root is None:
            return
        dx = root.get(index) - point.get(index)
        dis = distance(root._x, root._y, point._x, point._y)
        if dis < radius:
            self._nodes_in_radius.append(root)
        index = (index + 1) % self._dim
        self.radius_rec(root.left if dx > 0 else root.right, point, index, radius)
        if abs(dx) >= radius:
            return
        self.radius_rec(root.right if dx > 0 else root.left, point, index, radius)

    def radius_search(self, point, radius):
        if self.parent is None:
            return
        self._nodes_in_radius = []
        self.radius_rec(self.parent, point, 0, radius)
        # if len(self._nodes_in_radius) > 0:
        #     print(point)
        #     print(len(self._nodes_in_radius), *self._nodes_in_radius)
        return self._nodes_in_radius


class Tree(object):
    def __init__(self, root):
        self.root = root
        self._nodes = [root]

    def add_node(self, node):
        self._nodes.append(node)


def distance(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
