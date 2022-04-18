import random
from math import atan2
from helper import *


# Get truth odometry from prev_state to next_state
def ground_truth_odo(prev_state, next_state):
    delta_x = next_state[0] - prev_state[0]
    delta_y = next_state[1] - prev_state[1]
    delta_rot1 = atan2(delta_y, delta_x) - prev_state[2]
    delta_trans = math.sqrt(delta_x**2+delta_y**2)
    delta_rot2 = next_state[2] - prev_state[2] - delta_rot1

    return (delta_rot1, delta_trans, delta_rot2)


# Generate normal sampling odometry from original odometry
def normal_sample_odo(truth_odo, sigma_rot1=0.05, sigma_rot2=0.05, sigma_trans=0.1):
    delta_prime_rot1 = np.random.normal(truth_odo[0], sigma_rot1)
    delta_prime_trans = np.random.normal(truth_odo[1], sigma_trans)
    delta_prime_rot2 = np.random.normal(truth_odo[2], sigma_rot2)

    return (delta_prime_rot1, delta_prime_trans, delta_prime_rot2)


# Get truth bearing observation with landmark
def bearing_observation(landmarks, robot_state):
    theta = robot_state[2]%(2*math.pi)

    obs = []
    for l in landmarks:
        delta_x = l[0] - robot_state[0]
        delta_y = l[1] - robot_state[1]
        n_theta = atan2(delta_y, delta_x) - theta
        n_theta = normal_radian(n_theta)
        obs.append(n_theta)

    return obs


# Generate observing sampling bearing observation from truth observation
def normal_sample_bearing(bearing_obs, sigma_land=math.pi/60):
    noraml_bs = []
    for b in bearing_obs:
        noraml_bs.append(normal_radian(np.random.normal(b, sigma_land)))

    return noraml_bs


# Generating a measurement file with error
def generate_result_file(states, K, landmarks, file_name):
    count = 0
    with open(file_name, 'w') as f:
        start_state = states[0]
        f.write("{} {} {}\n".format(start_state[0], start_state[1], start_state[2]))
        f.write("{}\n".format(K))
        afterward_states = states[1:]
        for i in range(len(afterward_states)):
            count += 1
            odo = ground_truth_odo(start_state, afterward_states[i])
            normal_odo = normal_sample_odo(odo)
            f.write("{} {} {}\n".format(normal_odo[0], normal_odo[1], normal_odo[2]))
            bears = bearing_observation(landmarks, afterward_states[i])
            normal_bears = normal_sample_bearing(bears)
            for b in normal_bears:
                f.write("{} ".format(b))
            f.write("\n")


# SIR Particle Filter
def particle_filter(S_prev, U_curr, Z_curr, landmarks, K):
    # Propagate particles by normal distributed sampling noisy odometry on time = 1
    S_1 = []
    for s in S_prev:
        new_state = sample_motion_model(U_curr, s)
        S_1.append(((new_state), s[1]))


    # Update weights by observation
    S_1_prime = []
    for s in S_1:
        w = s[1]
        w_sum = 0
        for idx, l in enumerate(landmarks):
            d_x = l[0] - s[0][0]
            d_y = l[1] - s[0][1]
            b_bar = normal_radian(math.atan2(d_y, d_x) - s[0][2])
            w *= normal_pdf(b_bar, math.pi / 60, Z_curr[idx]) + 1.e-300

        S_1_prime.append((s[0], w))


    # Resampling from weighted particles
    ws = []
    S_1_rtn = []
    for i in S_1_prime:
        ws.append(i[1])

    for idx, s in enumerate(S_1_prime):
        if sum(ws) == 0:
            # Since choices from random library doesn't take sum of weight equal to 0, I use this filter out those.
            continue
        else:
            x = np.random.normal((random.choices(S_1_prime, weights=ws)[0][0][0]), 2)
            y = np.random.normal((random.choices(S_1_prime, weights=ws)[0][0][1]), 2)
            r = np.random.normal((random.choices(S_1_prime, weights=ws)[0][0][2]), math.pi / 60)
            wt = random.choices(S_1_prime, weights=ws)[0][1]

            S_1_rtn.append(((x,y,r),wt))

    return S_1_rtn


# Calculate distributed robot motion.
def sample_motion_model(noisy_odo, state_s, sigma_rot1=0.05, sigma_rot2=0.05, sigma_trans=0.1):
    d_rot1 = random.gauss(noisy_odo[0], sigma_rot1)
    d_tran = random.gauss(noisy_odo[1], sigma_trans)
    d_rot2 = random.gauss(noisy_odo[2], sigma_rot2)
    guess_odo = (d_rot1, d_tran, d_rot2)

    new_x = state_s[0][0] + guess_odo[1] * math.cos(state_s[0][2] + guess_odo[0])
    new_y = state_s[0][1] + guess_odo[1] * math.sin(state_s[0][2] + guess_odo[0])
    new_r = state_s[0][2] + guess_odo[0] + guess_odo[2]

    return (new_x, new_y, new_r)