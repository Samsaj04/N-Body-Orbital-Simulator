import numpy as np

sun_mass =  1.9891e30
mag = lambda v: np.linalg.norm(v)

def R_soi(d, m):
    return d * (m/sun_mass)**(2/5)

def D3(x):
    if x.shape == (2,):
        return np.array([x[0], x[1], 0.0])
    return x

def h_vec(r, v):
    return np.cross(D3(r), D3(v))

def epsilon(r, v, mu):
    return (mag(v)**2 / 2) - mu/mag(r)

def eccent(r, v, mu):
    e_vec = np.cross(D3(v), h_vec(r, v))/mu - D3(r)/mag(r)
    e = mag(e_vec)
    return e, e_vec/e

def periap(r, v, mu):
    return mag(h_vec(r,v))**2 / (mu*(1+eccent(r,v, mu)[0]))

def dV_hyper(r, v, mu):
    Rp = periap(r,v,mu)
    Vp = np.sqrt(2 * (epsilon(r, v, mu) + mu/Rp))
    Vc = np.sqrt(mu/Rp)
    V_hat = np.cross(h_vec(r, v), eccent(r,v,mu)[1])
    V_hat /= mag(V_hat)
    return (Vc - Vp) * np.abs(V_hat)

def T_hyper(t0, r, v, mu):
    e = eccent(r,v,mu)[0]
    e_vec = eccent(r,v,mu)[1]*e
    
    nu = np.acos(np.dot(e_vec, D3(r))/(e*mag(r)))
    nu = nu if np.dot(D3(r), D3(v)) >= 0 else -nu
    a = -mu/(2*epsilon(r, v, mu))
    F = 2*np.arctanh(np.sqrt((e-1)/(e+1)) * np.tan(nu/2))
    return t0 - np.sqrt(-a**3/mu) * (e*np.sinh(F)-F)
