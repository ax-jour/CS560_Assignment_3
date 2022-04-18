from file_parse import *
from visualizer import *
from robot import *
from ground_truth import *
from landmark import *
from localization import *


if __name__ == '__main__':

    N = 10 # number of landmarks
    K = 240 # number of truth trajectories
    file_name = 2 # name of txt file

    # # Generate landmarks file.
    # generate_landmarks(N, './landmark_file/{}.txt'.format(file_name))

    # Load landmarks object
    landmarks = parse_landmarks('./landmark_file/{}.txt'.format(file_name))

    # Robot object
    start_state = (60.0, 70.0, -math.pi/2)
    rob = robot(8, 4)
    rob.set_pose(start_state)

    # # Visualize landmarks
    # plt.figure()
    # visualize_landmarks(landmarks)
    # plot_robot((0,0), rob.transform())
    # plt.show()

    # # Generate rectangular ground truth traj.
    # rectangular_path(K, start_state, 35, './ground_truth_file/{}.txt'.format(file_name))

    # Load ground truth traj
    states = parse_traj('./ground_truth_file/{}.txt'.format(file_name))

    # # Visualize landmarks and robot ground truth traj
    # plt.figure()
    # rob.set_pose(states[0])
    # plot_robot((0, 0), rob.transform())
    # visualize_landmarks(landmarks)
    # for i in states:
    #     rob.set_pose(i)
    #     plot_robot((0,0), rob.transform())
    # plt.show()

    # Generating observation file
    generate_result_file(states, K, landmarks, './observation_file/{}.txt'.format(file_name))

    # Load observation file
    X_0, ctl_num, odo_mms, obs_mss = parse_measurement('./observation_file/{}.txt'.format(file_name))

    # Generate Set of particles on time = 0
    num_p = 1000
    S_0 = []
    for i in range(num_p):
        S_0.append(((random.uniform(0,100),random.uniform(0,100),random.uniform(-math.pi,math.pi)), 1))

    # Visualization of each time t.
    plt.figure()
    visualize_landmarks(landmarks)
    for j in range(K):
        S_1 = particle_filter(S_0, odo_mms[j], obs_mss[j], landmarks, K)

        rob.set_pose(states[j])
        plot_robot((0,0), rob.transform())
        visualize_landmarks(landmarks)
        v_points = []
        for i in S_1:
            if i[1] == 0:
                continue
            v_points.append(i)
        visualize_particles(v_points)
        plt.show()






