"""Point and tree classes."""


class Point(object):
    """Standard point class."""

    def __init__(self, cor_x, cor_y):
        """Init point coordinates."""
        self.cor_x = cor_x
        self.cor_y = cor_y

    def get(self, index):
        """Get coordinate by index."""
        if index == 0:
            return self.cor_x
        if index == 1:
            return self.cor_y


class Node(object):
    """Node for KD tree."""

    def __init__(self, cor_x=0, cor_y=0):
        """Init node structure."""
        self.cor_x = cor_x
        self.cor_y = cor_y
        self.child = []
        self.parent = None
        self.left = None
        self.right = None
        self._same = []
        self.cost = 0
        self.depth = 0
        self.eps = 0.0001

    def add_child(self, node):
        """Add child to node."""
        self.child.append(node)

    def add_same(self, node):
        """Adding node with same coordinates."""
        self._same.append(node)

    def define_parent(self, parent):
        """Define or redefine parent node."""
        self.parent = parent
        self.depth = parent.depth + 1
        dis = distance(self.cor_x, self.cor_y, parent.cor_x, parent.cor_y)
        self.cost = parent.cost + dis

    def get(self, index):
        """Get coordinate by index."""
        if index == 0:
            return self.cor_x
        return self.cor_y

    def delete_child(self, node):
        """Delete child node."""
        for index, child in enumerate(self.child):
            if (
                abs(child.cor_x - node.cor_x) < self.eps
                and abs(child.cor_y - node.cor_y) < self.eps
            ):
                self.child[index].parent = None
                self.child.pop(index)

    def __str__(self):
        """Represent string notion."""
        return '({0} {1})'.format(self.cor_x, self.cor_y)


class KDTree(object):
    """KD tree class."""

    def __init__(self, parent):
        """Init KD tree."""
        self.parent = parent
        self._best = None
        self._best_dist = 0
        self._dim = 2
        self.visited = 0
        self._nodes_in_radius = []

    def insert_rec(self, point, node, s_dim):
        """Recursive insertion of new node."""
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
        """Insert new node."""
        return self.insert_rec(point, root, 0)

    def nearest_rec(self, root, point, index):
        """Recursive search of the nearest node."""
        if root is None:
            return
        dis = distance(root.cor_x, root.cor_y, point.cor_x, point.cor_y)
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
        """Search of the nearest node."""
        if self.parent is not None:
            self._best = None
            self._best_dist = 0
            self.nearest_rec(self.parent, point, 0)
            return self._best

    def radius_rec(self, root, point, index, radius):
        """Recursive search of nodes in radius."""
        if root is None:
            return
        self.visited += 1
        dx = root.get(index) - point.get(index)
        dis = distance(root.cor_x, root.cor_y, point.cor_x, point.cor_y)
        if dis < radius:
            self._nodes_in_radius.append(root)
        index = (index + 1) % self._dim
        self.radius_rec(
            root.left if dx > 0 else root.right, point, index, radius,
        )
        if abs(dx) >= radius:
            return
        self.radius_rec(
            root.right if dx > 0 else root.left, point, index, radius,
        )

    def radius_search(self, point, radius):
        """Search of nodes in radius."""
        if self.parent is not None:
            self.visited = 0
            self._nodes_in_radius = []
            self.radius_rec(self.parent, point, 0, radius)
            return self._nodes_in_radius


class Tree(object):
    """Tree with multiple child."""

    def __init__(self, root):
        """Tree init."""
        self.root = root


def distance(x1, y1, x2, y2):
    """Calculate euclidean distance."""
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
