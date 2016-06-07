import random
import math
from Maze import Maze


class RRTPoint:
    def __init__(self, x, y, parent=None):
        self.x = int(x)
        self.y = int(y)
        self.parent = parent

    def __str__(self):
        return "<x=" + str(self.x) + ", y=" + str(self.y) + ">"


class RRTMaze(Maze):
    def __init__(self, polygons, width, height, start, end):
        Maze.__init__(self, polygons, width, height, 1, RRTPoint(start[0], start[1]), RRTPoint(end[0], end[1]))
        self.max_distance = 150
        self.min_distance = 50
        self.target_size = 15

    def generate_random_grid_point(self):
        x = random.randint(0, self.pixel_width)
        y = random.randint(0, self.pixel_height)
        return RRTPoint(x, y, None)

    def find_closest_tree_point(self, point, tree):
        smallest_distance = float("inf")
        best_index = -1

        for x in xrange(0, len(tree)):
            current_distance = self.calculate_distance(point, tree[x])
            if current_distance < smallest_distance:
                smallest_distance = current_distance
                best_index = x

        if best_index == -1:
            return None
        else:
            return tree[best_index]

    def generate_distance_scaled_point(self, tree):
        random_point = self.generate_random_grid_point()
        tree_point = self.find_closest_tree_point(random_point, tree)
        distance = self.calculate_distance(random_point, tree_point)

        if distance > self.max_distance:
            random_point.x -= tree_point.x
            random_point.y -= tree_point.y
            random_point.x = int(random_point.x * (self.max_distance / distance))
            random_point.y = int(random_point.y * (self.max_distance / distance))
            random_point.x += tree_point.x
            random_point.y += tree_point.y

        return tree_point, random_point

    def test_from_grid(self, x, y):
        return self.is_traversable(x, y)

    def point_is_in_tree(self, tree, point):
        for tree_point in tree:
            if point.x == tree_point.x and point.y == tree_point.y:
                return True
        return False

    def generate_traversable_grid_point(self, current_tree):
        while True:
            parent, new_point = self.generate_distance_scaled_point(current_tree)

            # test to see if the point has already been added to the tree
            if self.point_is_in_tree(current_tree, new_point):
                continue

            # if the new point is not reachable from the tree, throw it away
            if not self.straight_line_reachable(parent, new_point):
                continue

            # if the point is new and traversable, return it
            return parent, new_point

    def straight_line_reachable(self, from_point, to_point):
        if from_point.x < to_point.x:
            smaller_point = from_point
            larger_point = to_point
        else:
            smaller_point = to_point
            larger_point = from_point

        delta_x = larger_point.x - smaller_point.x

        if delta_x == 0:
            if smaller_point.y < larger_point.y:
                smaller_y = smaller_point.y
                larger_y = larger_point.y
            else:
                smaller_y = larger_point.y
                larger_y = smaller_point.y

            for y in xrange(int(smaller_y), int(larger_y) + 1):
                if not self.test_from_grid(smaller_point.x, y):
                    return False

        else:
            slope = float(larger_point.y - smaller_point.y) / (larger_point.x - smaller_point.x)

            if abs(slope) < 1:

                for x in xrange(1, int(larger_point.x - smaller_point.x + 1)):

                    if not self.test_from_grid(smaller_point.x + x, int(smaller_point.y + x * slope)):
                        return False
            else:
                if smaller_point.y < larger_point.y:
                    smaller_y = smaller_point.y
                    larger_y = larger_point.y
                else:
                    smaller_y = larger_point.y
                    larger_y = smaller_point.y
 
                b = int(smaller_point.y - (slope * smaller_point.x))

                for y in xrange(smaller_y, larger_y + 1):
                    new_x = int((y - b) / slope)
                    if not self.test_from_grid(new_x, y):
                        return False
        
        return True

    def generate_path(self):
        # setup tree
        destination = self.end
        tree = []
        tree.append(self.start)

        while True:
            parent, new_point = self.generate_traversable_grid_point(tree)
            new_point.parent = parent

            print "ADDING: %s" % new_point
            tree.append(new_point)
            if self.calculate_distance(new_point, self.end) < self.target_size:
                break

        # generate result path
        result_point_sequence = []
        current_waypoint = tree[len(tree) - 1]
        while current_waypoint is not None:
            result_point_sequence.append((current_waypoint.x, current_waypoint.y))
            current_waypoint = current_waypoint.parent

        return list(reversed(result_point_sequence))

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)



class TestPoly:
    def __init__(self):
        self.points = [
            RRTPoint(2, 2),
            RRTPoint(40, 40),
            RRTPoint(2, 40),
            RRTPoint(40, 2)
        ]

if __name__ == "__main__":

    test_case_string = \
    """
    000000000000
    011111111110
    010000000010
    011011010010
    011011011310
    011011011010
    011011000010
    010000000010
    012001011110
    011101011110
    011111111110
    000000000000
    """

    start = None
    end = None

    data = []
    lines = str.split(test_case_string)
    for x, line in enumerate(lines):
        bool_line = []
        for y, character in enumerate(line):
            bool_line.append(character != "1")

            if character == "2":
                start = (x, y)

            if character == "3":
                end = (x, y)
        data.append(bool_line)


    poly = TestPoly()
    new_maze = RRTMaze([poly], 12, 12, start, end)
    new_maze.grid = data
    new_maze.max_distance = 1
    new_maze.target_size = 1

    for line in new_maze.grid:
        print line

    print new_maze.straight_line_reachable(RRTPoint(2, 7), RRTPoint(7, 7))
