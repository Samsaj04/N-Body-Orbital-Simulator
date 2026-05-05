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

def T_hohmann(r1, r2, mu):
    return T(r1, mu), T(r2, mu)

def T_biell(r1, r2, r3, mu):
    T0, _ = T_hohmann(r1, r2, mu)
    _, T2 = T_hohmann(r2, r3, mu)
    return T0, T2

#================================================

def circular_HT(r1, r2, mu):
    hT = h_T(r1, r2, mu)
    dV1 = (hT - h(r1, 0, mu))/r1
    dV2 = (h(r2, 0, mu) - hT)/r2
    return dV1, dV2, T((r1+r2)/2, mu)

def elliptic_HT(r1, a1, r2, a2, mu):
    a_T = (r1+r2)/2
    dVA = V_orbit(r1, a_T, mu) - V_orbit(r1, a1, mu)
    dVB = V_orbit(r2, a2, mu) - V_orbit(r2, a_T, mu)
    return dVA, dVB, T(a_T, mu)

def bi_elliptic(r1, r2, r3, mu):
    h2T = h_T(r1, r2, mu) 
    dV1 = (h2T - h(r1, 0, mu))/r1
    dV2 = (h_T(r2, r3, mu) - h2T)/r2
    return dV1, dV2, T((r1+r2)/2, mu), T((r2+r3)/2, mu)

def h(a, e, mu):
    return np.sqrt(mu*a*(1-e**2))

def h_T(r1, r2, mu):
    return np.sqrt(2*mu*r1*r2/(r1+r2))