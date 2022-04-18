import matplotlib.pyplot as plt


# Visualize boundaries and landmarks.
def visualize_landmarks(landmarks):
    boundary = [[(0, 0), (0, 100)], [(0, 100), (100, 100)], [(100, 100), (100, 0)], [(100, 0), (0, 0)]]
    xob, yob = zip(*boundary)
    plt.plot(xob, yob, 'k')

    for i in landmarks:
        plt.plot(i[0], i[1], 'bo')


# Visualize boundaries and landmarks.
def visualize_particles(particles):
    boundary = [[(0, 0), (0, 100)], [(0, 100), (100, 100)], [(100, 100), (100, 0)], [(100, 0), (0, 0)]]
    xob, yob = zip(*boundary)
    plt.plot(xob, yob, 'k')

    for i in particles:
        plt.plot(i[0][0], i[0][1], 'go')


# Visualize the robot in chart
def plot_robot(point, robot):
    robot_in_place = []
    for pt in robot:
        pt = list(pt)
        pt[0] = pt[0] + point[0]
        pt[1] = pt[1] + point[1]
        robot_in_place.append(tuple(pt))
    robot_in_place.append(robot_in_place[0])
    xss, yss = zip(*robot_in_place)
    plt.plot(xss, yss, 'y')

    xf, yf = zip(*[robot_in_place[2], robot_in_place[3]])
    plt.plot(xf, yf, 'r')


# Visualize the robot in chart
def plot_robot_v2(point, robot):
    robot_in_place = []
    for pt in robot:
        pt = list(pt)
        pt[0] = pt[0] + point[0]
        pt[1] = pt[1] + point[1]
        robot_in_place.append(tuple(pt))
    robot_in_place.append(robot_in_place[0])
    xss, yss = zip(*robot_in_place)
    plt.plot(xss, yss, 'g')

    xf, yf = zip(*[robot_in_place[2], robot_in_place[3]])
    plt.plot(xf, yf, 'r')
