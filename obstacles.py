class Polygon(object):
    def __init__(self):
        self.points = []

    def add_point(self, point):
        self.points.append(point)

    def check_inside(self, point):
        size = len(self.points)
        counter = 0
        p1 = self.points[0]
        for i in range(1, size + 1):
            p2 = self.points[i % size]
            if point._y > min(p1._y, p2._y):
                if point._y <= max(p1._y, p2._y):
                    if point._x <= max(p1._x, p2._x):
                        if p1._y != p2._y:
                            xinters = (point._y - p1._y) * (p2._x - p1._x) / (
                                    p2._y - p1._y) + p1._x
                            if p1._x == p2._x or point._x <= xinters:
                                counter += 1
            p1 = p2
        if counter % 2 == 0:
            return False
        else:
            return True

    def segment_obstacle(self, start, end):
        if self.check_inside(start) or self.check_inside(end):
            return True
        p1 = self.points[0]
        size = len(self.points)
        for i in range(1, size + 1):
            p2 = self.points[i % size]
            if segment_intersection(start, end, p1, p2):
                return True
            p1 = p2
        return False


def ccw(p1, p2, p3):
    return (p3._y - p1._y) * (p2._x - p1._x) > (p2._y - p1._y) * (p3._x - p1._x)


def segment_intersection(start1, end1, start2, end2):
    return ccw(start1, start2, end2) != ccw(end1, start2, end2) and \
           ccw(start1, end1, start2) != ccw(start1, end1, end2)
