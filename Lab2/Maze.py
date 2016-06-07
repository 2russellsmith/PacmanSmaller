class Maze:
    """
    polygons should be an array of polygons, each one representing an obstacle in the maze
    width is the pixel width of the maze
    height is the pixel height of the maze
    start is the location of the entrance to the maze
    end is the location of the exit of the maze
    """
    def __init__(self, polygons, width, height, granularity, start, end):
        self.grid = []
        self.polygons = polygons

        for polygon in polygons:
            self.expand_polygon(polygon)

        self.granularity = granularity
        self.start = start
        self.end = end
        self.pixel_width = width
        self.pixel_height = height

        '''
        for x in xrange(0, width / granularity):
            line = []
            for y in xrange(0, height / granularity):
                line.append(self.is_traversable(x, y))
            self.grid.append(line)
        '''

    def __str__(self):
        result = "polygons: " + str(self.polygons) + "\n"
        result += "granularity: " + str(self.granularity) + "\n"
        result += "start: " + str(self.start) + "\n"
        result += "end: " + str(self.end) + "\n"
        result += "width: " + str(self.pixel_width) + "\n"
        result += "height: " + str(self.pixel_height) + "\n"
        for line in self.grid:
            result += str(line) + "\n"
        return result

    def expand_polygon(self, polygon):
        scale_factor = 1.5
        average_x = sum([item.x for item in polygon.points]) / len(polygon.points)
        average_y = sum([item.y for item in polygon.points]) / len(polygon.points)

        for point in polygon.points:
            point.x -= average_x
            point.y -= average_y
            point.x *= scale_factor
            point.y *= scale_factor
            point.x += average_x
            point.y += average_y

    def polygon_contains_point(self, polygon, x, y):
        polygon_start_x = 1000000
        polygon_end_x = -1
        polygon_start_y = 1000000
        polygon_end_y = -1

        for point in polygon.points:
            if point.x < polygon_start_x:
                polygon_start_x = point.x

            if point.x > polygon_end_x:
                polygon_end_x = point.x

            if point.y < polygon_start_y:
                polygon_start_y = point.y

            if point.y > polygon_end_y:
                polygon_end_y = point.y

        return polygon_start_x <= x <= polygon_end_x and polygon_start_y <= y <= polygon_end_y

    def is_traversable(self, x, y):
        for polygon in self.polygons:
            if self.polygon_contains_point(polygon, x, y):
                print "HIT POLYGON %s" % polygon.points
                return False
        return True

    # this to be implemented in subclasses
    def generate_path(self):
        pass


class TestPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        pass

    def __str__(self):
        return "<x=" + str(self.x) + ", y=" + str(self.y) + ">"


class TestPoly:
    def __init__(self):
        self.points = [
            TestPoint(2, 2),
            TestPoint(40, 40),
            TestPoint(2, 40),
            TestPoint(40, 2)
        ]


if __name__ == "__main__":
    poly = TestPoly()
    new_maze = Maze([poly], 100, 100, 10, TestPoint(0, 0), TestPoint(100, 100))
    print new_maze
