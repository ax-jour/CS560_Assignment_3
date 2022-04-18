from helper import *


def parse_landmarks(landmark_file):
    f_landmark = open(landmark_file, 'r')
    landmarks = []
    for obsLine in f_landmark:
        x = obsLine.split()
        if len(x) < 2:
            continue
        coord = (float(x[0]), float(x[1]))
        landmarks.append(coord)

    return landmarks


def parse_traj(traj_file):
    f_traj = open(traj_file, "r")

    states = []
    for stateLine in f_traj:
        obs = str2floatInNestedLst(chunks(stateLine.split(), 3))
        if len(obs[0]) < 2:
            continue
        states.append(obs[0])
    return states

def parse_measurement(measurement_file):
    f_ms = open(measurement_file, "r")

    f_lines = f_ms.readlines()
    start_state = readState(f_lines[0].split())
    ctl_num = int(f_lines[1])
    odo_mss = []
    obs_mss = []
    for idx, val in enumerate(f_lines[2:]):
        line = val.split()
        if idx%2 == 0: # Even line: State odometry
            odo_mss.append(readState(line))
        else:
            obs_mss.append(readBearing(line))

    return start_state, ctl_num, odo_mss, obs_mss

