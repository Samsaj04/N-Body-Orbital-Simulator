import numpy as np

# Orbit Velocity
def V_orbit(rad, a, mu):
    return np.sqrt(mu * (2/rad - 1/a))

# Orbit Period
def T(a, mu):
    return 2*np.pi*np.sqrt(a**3 / mu)

def T_imp(r1, r2, mu):
    return T(r1, mu), T(r2, mu)

def hohmann_transfer(r1, r2, mu):
    a_trans = (r1+r2)/2
    dV1 = V_orbit(r1, a_trans, mu) - V_orbit(r1, r1, mu)
    dV2 = V_orbit(r2, r2, mu) - V_orbit(r2, a_trans, mu)
    return dV1, dV2, T(a_trans, mu)

def dV_tot(r1, r2, mu):
    dV1, dV2, _ = hohmann_transfer(r1, r2, mu)
    return dV1 + dV2