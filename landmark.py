import random
random.seed(1)

# Generate N number of landmarks randomly.
def generate_landmarks(N, file_name):
    with open(file_name, 'w') as f:
        f.write("{}\n".format(N))
        for i in range(N):
            x = random.randint(1, 99)
            y = random.randint(1, 99)
            f.write("{} {}\n".format(x, y))