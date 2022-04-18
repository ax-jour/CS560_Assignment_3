import numpy as np
import sys, math

## Global variables
env_boundaries = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

def EuclideanDist(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

## Divide a line to coordinate tuples.
def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def normal_pdf(mu, sigma, x):
    return math.e**((-1/2)*((x - mu)/sigma)**2)/(sigma*math.sqrt(2*math.pi))


def normal_radian(radian):
    while radian < 0:
        radian = radian + 2*math.pi
    while radian > 2*math.pi:
        radian = radian - 2*math.pi
    return radian


## Cast str to float in the 2D list.
def str2floatInNestedLst(nestedLst):
    rtnLst = []
    for sub in nestedLst:
        ls = []
        for e in sub:
            ls.append(float(e))
        rtnLst.append(tuple(ls))
    return rtnLst


def readState(line):
    float_ls = []
    for i in line:
        float_ls.append(float(i))
    return tuple(float_ls)


def readBearing(line):
    float_ls = []
    for i in line:
        float_ls.append(float(i))
    return float_ls


## Move robot to point's coordinate.
def robotPointState(robot, point):
    robOnStatePt = []
    for pt in robot:
        coord = []
        coord.append(round(pt[0] + point[0], 4))
        coord.append(round(pt[1] + point[1], 4))
        robOnStatePt.append(coord)

    return robOnStatePt


## Triangulation a polygon obstacle.
## Reference: https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
def triangulatePolys(obstacles):
    polyTriObs = []
    for obs in obstacles:
        polyTriObs.extend(polyTriangulate(obs))

    return polyTriObs

def polyTriangulate(obstacle):
    obs_points = []
    for obs in obstacle:
        obs_points.append(obs)

    if isClockwise(obs_points):
        obs_points.reverse()

    ear_vertex = []
    triangles = []
    point_count = len(obs_points)
    for i in range(point_count):
        prev_index = i - 1
        prev_point = obs_points[prev_index]
        point = obs_points[i]
        next_index = (i + 1) % point_count
        next_point = obs_points[next_index]

        if isEar(prev_point, point, next_point, obs_points):
            ear_vertex.append(point)

    while ear_vertex and point_count >= 3:
        ear = ear_vertex.pop(0)
        i = obs_points.index(ear)
        prev_index = i - 1
        prev_point = obs_points[prev_index]
        next_index = (i + 1) % point_count
        next_point = obs_points[next_index]

        obs_points.remove(ear)
        point_count -= 1
        triangles.append([[prev_point[0], prev_point[1]], [ear[0], ear[1]], [next_point[0], next_point[1]]])
        if point_count > 3:
            prev_prev_point = obs_points[prev_index - 1]
            next_next_index = (i + 1) % point_count
            next_next_point = obs_points[next_next_index]

            groups = [
                (prev_prev_point, prev_point, next_point, obs_points),
                (prev_point, next_point, next_next_point, obs_points),
            ]
            for group in groups:
                p = group[1]
                if isEar(*group):
                    if p not in ear_vertex:
                        ear_vertex.append(p)
                elif p in ear_vertex:
                    ear_vertex.remove(p)
    return triangles


def isClockwise(obstacle):
    s = 0
    obstacle_count = len(obstacle)
    for i in range(obstacle_count):
        point = obstacle[i]
        point2 = obstacle[(i + 1) % obstacle_count]
        s += (point2[0] - point[0]) * (point2[1] + point[1])
    return s > 0


def isEar(p1, p2, p3, obstacle):
    if containsNoPts(p1, p2, p3, obstacle):
        if isConvex(p1, p2, p3):
            if triangleArea(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]) > 0:
                return True
    return False


def containsNoPts(p1, p2, p3, obstacle):
    for pn in obstacle:
        if pn in (p1, p2, p3):
            continue
        elif isPtInside(pn, p1, p2, p3):
            return False
    return True


def isPtInside(p, a, b, c):
    area = triangleArea(a[0], a[1], b[0], b[1], c[0], c[1])
    area1 = triangleArea(p[0], p[1], b[0], b[1], c[0], c[1])
    area2 = triangleArea(p[0], p[1], a[0], a[1], c[0], c[1])
    area3 = triangleArea(p[0], p[1], a[0], a[1], b[0], b[1])
    areadiff = abs(area - sum([area1, area2, area3])) < math.sqrt(sys.float_info.epsilon)
    return areadiff


def isConvex(prev, point, next):
    return triangleSum(prev[0], prev[1], point[0], point[1], next[0], next[1]) < 0


def triangleArea(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)


def triangleSum(x1, y1, x2, y2, x3, y3):
    return x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1)

def isSAToverlap(p1, p2):
    p1 = [np.array(v, 'float64') for v in p1]
    p2 = [np.array(v, 'float64') for v in p2]

    edges = edges_of(p1)
    edges += edges_of(p2)
    orthogonals = [orthogonal(e) for e in edges]

    push_vectors = []
    for o in orthogonals:
        separates, pv = is_separating_axis(o, p1, p2)

        if separates:
            # they do not collide and there is no push vector
            return False, None
        else:
            push_vectors.append(pv)

    # they do collide and the push_vector with the smallest length is the MPV
    mpv =  min(push_vectors, key=(lambda v: np.dot(v, v)))

    # assert mpv pushes p1 away from p2
    d = centers_displacement(p1, p2) # direction from p1 to p2
    if np.dot(d, mpv) > 0: # if it's the same direction, then invert
        mpv = -mpv

    return True, mpv

def centers_displacement(p1, p2):
    # geometric center
    c1 = np.mean(np.array(p1), axis=0)
    c2 = np.mean(np.array(p2), axis=0)
    return c2 - c1

def edges_of(vertices):
    edges = []
    N = len(vertices)

    for i in range(N):
        edge = vertices[(i + 1)%N] - vertices[i]
        edges.append(edge)

    return edges

def orthogonal(v):
    return np.array([-v[1], v[0]])

def is_separating_axis(o, p1, p2):
    min1, max1 = float('+inf'), float('-inf')
    min2, max2 = float('+inf'), float('-inf')

    for v in p1:
        projection = np.dot(v, o)

        min1 = min(min1, projection)
        max1 = max(max1, projection)

    for v in p2:
        projection = np.dot(v, o)

        min2 = min(min2, projection)
        max2 = max(max2, projection)

    if max1 >= min2 and max2 >= min1:
        d = min(max2 - min1, max1 - min2)
        # push a bit more than needed so the shapes do not overlap in future
        # tests due to float precision
        pv = [0, 0]
        if np.dot(o, o) != 0:
            d_over_o_squared = d/np.dot(o, o) + 1e-10
            pv = d_over_o_squared*o
        return False, pv
    else:
        return True, None

def CloneList(li1):
    li_copy = li1[:]
    return li_copy

# TODO: Find vertices that outside of the new obstacles. and sort them.
def minkowski_add(poly1, obstacle):
    c_free_obs = []
    for pt1 in poly1:
        for pt2 in obstacle:
            new_pt = [pt1[0] + pt2[0], pt1[1] + pt2[1]]
            c_free_obs.append(new_pt)

    c_free_obs = sortVerticesClockwise(c_free_obs)
    return c_free_obs

def sortVerticesClockwise(polygon):
    mid_point = polygon[int(len(polygon)/2)]

    center_point = None
    if (mid_point[1] - polygon[0][1]) >= 0 and (mid_point[0] - polygon[0][0]) >= 0:
        center_point = (polygon[0][0] + (mid_point[0]-polygon[0][0])/2, polygon[0][1] + (mid_point[1]-polygon[0][1])/2)
    elif (mid_point[1] - polygon[0][1]) >= 0 and (mid_point[0] - polygon[0][0]) < 0:
        center_point = (polygon[0][0] - (mid_point[0]-polygon[0][0])/2, polygon[0][1] + (mid_point[1]-polygon[0][1])/2)
    elif (mid_point[1] - polygon[0][1]) < 0 and (mid_point[0] - polygon[0][0]) < 0:
        center_point = (polygon[0][0] - (mid_point[0]-polygon[0][0])/2, polygon[0][1] - (mid_point[1]-polygon[0][1])/2)
    elif (mid_point[1] - polygon[0][1]) < 0 and (mid_point[0] - polygon[0][0]) >= 0:
        center_point = (polygon[0][0] + (mid_point[0]-polygon[0][0])/2, polygon[0][1] - (mid_point[1]-polygon[0][1])/2)

    total_list = []

    sorted_vert1 = []
    for vert in polygon:
        x = vert[0] - center_point[0]
        y = vert[1] - center_point[1]
        v = dict()
        if x == 0 and y > 0:
            v['vert'] = vert
            v['k'] = 0
            sorted_vert1.append(v)

    total_list.extend(sorted_vert1)

    sorted_vert2 = []
    for vert in polygon:
        x = vert[0] - center_point[0]
        v = dict()
        if x > 0:
            v['vert'] = vert
            v['k'] = (vert[1] - center_point[1]) / (vert[0] - center_point[0])
            sorted_vert2.append(v)

    if len(sorted_vert2) > 0:
        sorted_vert2 = sorted(sorted_vert2, key=lambda x: x['k'], reverse=True)
        total_list.extend(sorted_vert2)

    sorted_vert3 = []
    for vert in polygon:
        x = vert[0] - center_point[0]
        y = vert[1] - center_point[1]
        v = dict()
        if x == 0 and y < 0:
            v['vert'] = vert
            v['k'] = 0
            sorted_vert3.append(v)

    total_list.extend(sorted_vert3)

    sorted_vert4 = []
    for vert in polygon:
        x = vert[0] - center_point[0]
        v = dict()
        if x < 0:
            v['vert'] = vert
            v['k'] = (vert[1] - center_point[1]) / (vert[0] - center_point[0])
            sorted_vert4.append(v)

    if len(sorted_vert4) > 0:
        sorted_vert3 = sorted(sorted_vert4, key=lambda x: x['k'], reverse=True)
        total_list.extend(sorted_vert3)


    # print(total_list)
    rtnList = []
    for ob in total_list:
        rtnList.append(tuple(ob['vert']))
    return rtnList

