
import numpy as np

def update1D(mu1, var1, mu2, var2):
    mu = ((mu1 * var2) + (mu2 * var1)) / (var1 + var2)
    var = 1. / ((1. / var1) + (1. / var2))
    return [mu, var]

def predict1D(mu1, var1, mu2, var2):
    mu = mu1 + mu2
    var = var1 + var2
    return [mu, var]

def predict(x, P, u, Q, F, B):
    x = F @ x + B @ u;
    P = F @ P @ F.T + Q
    return [x, P]

def update(x, P, z, R, H):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ S.I
    x = x + K @ y
    I = np.eye(len(x))
    P = (I - K@H) @ P
    return [x, P]

def run_1D_example():

    measurements = [5., 6., 7., 9., 10.]
    motions = [1., 1., 2., 1., 1.]

    measurement_noise = 4
    motion_noise = 2

    mu = 0.
    var = 10000.

    if len(measurements) == len(motions):

        for i in range(len(measurements)):
            [mu, var] = update(mu, var, measurements[i], measurement_noise)
            print('Update:   ', [mu, var])

            [mu, var] = predict(mu, var, motions[i], motion_noise)
            print('Predicte: ', [mu, var])

def run_2D_example():
    x = np.matrix('0.0 ; 0.0')
    P = np.matrix('1000.0, 0.0 ; 0.0, 1000.0')
    u = np.matrix('0.0')
    Q = np.matrix('0.0, 0.0 ; 0.0, 0.0')
    B = np.matrix('0.0 ; 0.0')
    F = np.matrix('1.0, 1.0 ; 0.0, 1.0')
    H = np.matrix('1.0, 0.0')
    R = np.matrix('1.0')

    measurements = [1, 2, 3]

    for i in range(len(measurements)):
        z = np.asmatrix(measurements[i])
        [x, P] = predict(x, P, u, Q, F, B)
        [x, P] = update(x, P, z, R, H)
        
        print('x: ', x)
        print('P: ', P)


if __name__ == "__main__":
    # run_1D_example()
    run_2D_example()
