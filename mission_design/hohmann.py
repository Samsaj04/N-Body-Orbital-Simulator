import numpy as np

# Orbit Velocity
def V_orbit(rad, a, mu):
    try:
        v = np.sqrt(mu * (2/rad - 1/a))
        return v
    except:
        raise ValueError("Radius cannot exceed twice the semi-major axis (r > 2a) for orbital velocity.")

# Orbit Period
def T(a, mu):
    return 2*np.pi*np.sqrt(a**3 / mu)

def T_imp(r1, r2, mu):
    return T(r1, mu), T(r2, mu)

def circular_HT(r1, r2, mu):
    a_T = (r1+r2)/2
    dV1 = V_orbit(r1, a_T, mu) - V_orbit(r1, r1, mu)
    dV2 = V_orbit(r2, r2, mu) - V_orbit(r2, a_T, mu)
    return dV1, dV2, T(a_T, mu)

def elliptic_HT(r1, a1, r2, a2, mu):
    a_T = (r1+r2)/2
    dVA = V_orbit(r1, a_T, mu) - V_orbit(r1, a1, mu)
    dVB = V_orbit(r2, a2, mu) - V_orbit(r2, a_T, mu)
    return dVA, dVB, T(a_T, mu)