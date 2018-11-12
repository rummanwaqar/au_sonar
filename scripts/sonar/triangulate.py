import numpy as np
from itertools import combinations


class Line(object):
    def __init__(self, loc, heading):
        self.loc = loc
        self.heading = heading
        self.slope = np.tan(np.deg2rad(heading))
        self.intercept = loc[1] - (self.slope * loc[0])

    def intersection(self, other_line):
        # solving simultaneous equations
        a = np.array([[-self.slope, 1], [-other_line.slope, 1]])
        b = np.array([self.intercept, other_line.intercept])
        x = np.linalg.solve(a, b)
        return x

    def __str__(self):
        return 'y={}x + {}'.format(self.slope, self.intercept)


# list of tuples of format (x,y,angle) angle in degrees
def triangulate(data):
    if len(data) < 2:
        return
    intersections = []
    lines = [Line((x[0], x[1]), x[2]) for x in data]
    for pair in combinations(lines, 2):
        intersections.append(pair[0].intersection(pair[1]))
    result = np.mean(intersections, axis=0)
    return intersections, result


if __name__ == '__main__':
    data = [(0.087, -0.250, 59.728), (0.5, -0.273, 62.807), (0.97, -0.22,
                                                             75.798)]
    intersections, result = triangulate(data)
    print(result)
