import math


def rectangular_path(K, start, straight_len, file_name):
    with open(file_name, 'w') as f:
        f.write("{}\n".format(K+1))
        f.write("{} {} {}\n".format(start[0], start[1], start[2]))

        if K%8 != 0:
            print('The K number cannot make the robot back to x_0 state')
            return False

        for i in range(int(K/8)):
            state1 = (start[0] + straight_len * math.cos(start[2]),
                      start[1] + straight_len * math.sin(start[2]),
                      start[2])
            f.write("{} {} {}\n".format(state1[0], state1[1], state1[2]))
            state2 = (state1[0], state1[1], state1[2]-math.pi/2)
            f.write("{} {} {}\n".format(state2[0], state2[1], state2[2]))
            state3 = (state2[0] + straight_len * math.cos(state2[2]),
                      state2[1] + straight_len * math.sin(state2[2]),
                      state2[2])
            f.write("{} {} {}\n".format(state3[0], state3[1], state3[2]))
            state4 = (state3[0], state3[1], state3[2]-math.pi/2)
            f.write("{} {} {}\n".format(state4[0], state4[1], state4[2]))
            state5 = (state4[0] + straight_len * math.cos(state4[2]),
                      state4[1] + straight_len * math.sin(state4[2]),
                      state4[2])
            f.write("{} {} {}\n".format(state5[0], state5[1], state5[2]))
            state6 = (state5[0], state5[1], state1[2]+math.pi/2)
            f.write("{} {} {}\n".format(state6[0], state6[1], state6[2]))
            state7 = (state6[0] + straight_len * math.cos(state6[2]),
                      state6[1] + straight_len * math.sin(state6[2]),
                      state6[2])
            f.write("{} {} {}\n".format(state7[0], state7[1], state7[2]))
            state8 = (state7[0], state7[1], start[2])
            f.write("{} {} {}\n".format(state8[0], state8[1], state8[2]))