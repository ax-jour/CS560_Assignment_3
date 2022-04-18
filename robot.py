import numpy as np
import math

class robot:
    # Initial data structure.
    def __init__(self, height, width):
        self.width = height
        self.height = width
        self.translation = (0, 0)
        self.rotation = 0.0

    # Input (x coord, y coord, r angle) to set pose
    def set_pose(self, pose): # Input (x, y, r)
        self.translation = (pose[0], pose[1])
        self.rotation = pose[2]

    # Make the robot transform and returns coordinates of it's vertices.
    def transform(self):
        w = self.width
        l = self.height
        x = self.translation[0]
        y = self.translation[1]
        r = self.rotation

        # This is standard rotation term.
        std_rotation = [[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]]

        A = np.dot(std_rotation, [[-w/2, -w/2, w/2, w/2], [-l/2, l/2, l/2, -l/2]])
        B = A + np.matrix([[x, x, x, x], [y, y, y, y]])

        rob_ls = []
        geo_rob = list(B.T)
        for r in geo_rob:
            rob_ls.extend(r.tolist())

        return rob_ls

