import numpy as np
from scipy.integrate import odeint

def computeInterMatrix(Z, sd0):
    L = np.zeros((8,2))
    for idx in range(4):
        x = sd0[2*idx, 0]
        y = sd0[2*idx+1, 0]
        L[2*idx] = np.array([x/Z,y])
        L[2*idx+1] = np.array([y/Z,-x])
    return L

def img_rp(s, t, w):
    Z0 = s[-1]
    s = np.array([s[:-1]]).T
    # print(s)
    L = computeInterMatrix(Z0, s)
    return np.hstack((L @ w, w[0]))

def prediction(s0, w):
    w = w.reshape((3,2))
    for w0 in w:
        s0 = odeint(img_rp, s0, [0, 0.1], args=(w0,))
        s0 = s0[-1,:]
    print(s0)

w = np.array([0.1,0,0.1,0,0.1,0])
sp = np.array([-1,1,1,1,1,-1,-1,-1,0.5])
prediction(sp, w)