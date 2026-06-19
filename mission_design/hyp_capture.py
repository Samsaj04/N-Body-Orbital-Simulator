import numpy as np

sun_mass =  1.9891e30
mag = lambda v: np.linalg.norm(v)

# Sphere of Influence
def R_soi(d, m):
    return d * (m/sun_mass)**(2/5)

def D3(x):
    if x.shape == (2,):
        return np.array([x[0], x[1], 0.0])
    return x

# Specific Angular Momentum Vector
def h_vec(r, v):
    return np.cross(D3(r), D3(v))

# Specific Mechanical Energy
def epsilon(r, v, mu):
    return (mag(v)**2 / 2) - mu/mag(r)

# Eccentricity magnitude and Unitary Vector
def eccent(r, v, mu):
    e_vec = np.cross(D3(v), h_vec(r, v))/mu - D3(r)/mag(r)
    e = mag(e_vec)
    return e, e_vec/e

# Compute the Periapsis Radius Magnitude of an Orbit
def periap(r, v, mu):
    return mag(h_vec(r,v))**2 / (mu*(1+eccent(r,v, mu)[0]))

# Computes the dV to go from Hyperbolic orbit to circular orbit with R = R_periapsis
def dV_hyper(r, v, mu):
    Rp = periap(r,v,mu)
    Vp = np.sqrt(2 * (epsilon(r, v, mu) + mu/Rp))
    Vc = np.sqrt(mu/Rp)
    V_hat = np.cross(h_vec(r, v), eccent(r,v,mu)[1])
    V_hat /= mag(V_hat)
    return (Vc - Vp) * V_hat

# Time from SOI to Periapsis
def T_hyper(t0, r, v, mu):
    e = eccent(r,v,mu)[0]
    e_vec = eccent(r,v,mu)[1]*e
    
    nu = np.acos(np.dot(e_vec, D3(r))/(e*mag(r)))
    nu = nu if np.dot(D3(r), D3(v)) >= 0 else -nu
    a = -mu/(2*epsilon(r, v, mu))
    F = 2*np.arctanh(np.sqrt((e-1)/(e+1)) * np.tan(nu/2))
    return t0 - np.sqrt(-a**3/mu) * (e*np.sinh(F)-F)

# Hyperbolic Excess Velocity
def hyp_excess_speed(Rp_sc, Rp_p1, Rp_p2, mu):
    mu_sun = 1.32712440018e11
    a_T = (Rp_p1 + Rp_p2) / 2
    V_inf = np.sqrt(mu_sun * (2/Rp_p1 - 1/a_T)) - np.sqrt(mu_sun/Rp_p1)
    V_circ = np.sqrt(mu/Rp_sc)
    V_peri = np.sqrt(V_inf**2 + 2*mu/Rp_sc)
    return V_peri - V_circ
