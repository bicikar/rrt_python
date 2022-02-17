"""Polygon classes and intersection functions."""


class Polygon(object):
    """Main obstacle class. Points are indicated clockwise."""

    def __init__(self):
        """Init polygon points."""
        self.points = []

    def add_point(self, point):
        """Add point to polygon."""
        self.points.append(point)

    def check_inside(self, point):
        """Check if a given point is inside polygon."""
        size = len(self.points)
        counter = 0
        p1 = self.points[0]
        for it in range(1, size + 1):
            p2 = self.points[it % size]
            if point.cor_y > min(p1.cor_y, p2.cor_y):
                if point.cor_y <= max(p1.cor_y, p2.cor_y):
                    if point.cor_x <= max(p1.cor_x, p2.cor_x):
                        if p1.cor_y != p2.cor_y:
                            xinters = (point.cor_y - p1.cor_y) * (
                                p2.cor_x - p1.cor_x
                            ) / (
                                p2.cor_y - p1.cor_y
                            ) + p1.cor_x
                            if p1.cor_x == p2.cor_x or point.cor_x <= xinters:
                                counter += 1
            p1 = p2
        return counter % 2 != 0

    def segment_obstacle(self, start, end):
        """Check if given segment intersects the polygon."""
        if self.check_inside(start) or self.check_inside(end):
            return True
        p1 = self.points[0]
        size = len(self.points)
        for it in range(1, size + 1):
            p2 = self.points[it % size]
            if segment_intersection(start, end, p1, p2):
                return True
            p1 = p2
        return False


def ccw(p1, p2, p3):
    """Check clockwise order."""
    return (p3.cor_y - p1.cor_y) * (p2.cor_x - p1.cor_x) > (
        p2.cor_y - p1.cor_y
    ) * (p3.cor_x - p1.cor_x)


def segment_intersection(start1, end1, start2, end2):
    """Return true if line segments intersect."""
    return ccw(start1, start2, end2) != ccw(end1, start2, end2) and ccw(
        start1, end1, start2,
    ) != ccw(start1, end1, end2)
