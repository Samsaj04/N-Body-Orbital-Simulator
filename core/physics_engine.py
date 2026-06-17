import numpy as np
from scipy.integrate import solve_ivp
from numba import njit

@njit
def accel_numba(S, N, dim, G, masses):
    Pitt = np.zeros((int(N), int(dim)))
    S_og = S.reshape((2, int(N), int(dim)))
    
    pos = S_og[0]
    vel = S_og[1]
    
    for i in range(N):
        for j in range(N):
            if j != i:
                Pitt[i] += G * masses[j]/(np.linalg.norm(pos[j] - pos[i])**3) * (pos[j] - pos[i])
            else:
                continue
    return np.concatenate((vel.ravel(), Pitt.ravel()))

class PhysicsEngine:
    def __init__(self, bodies, G, masses, dim):
        self.G = np.float64(G)
        self.bodies = bodies
        self.masses = np.asarray(masses, dtype=np.float64)
        self.dim = dim
        self.N = len(masses)
    
    def accel(self, t, S):
        return accel_numba(S, self.N, self.dim, self.G, self.masses)
    
    def solver(self, state, ti, tf, step):
        sol0 = solve_ivp(
            fun=self.accel,
            t_span=(ti, tf),
            y0=np.asarray(state, dtype=np.float64),
            t_eval=np.linspace(ti, tf, step),
            method='DOP853',
            rtol=1e-6, 
            atol=1e-8)
        return sol0
    
    def add_impulse_(self, solution, ti, tf, dV, step):
        
        state_burn = solution[:, -1].copy()
        state_burn[-int(self.dim)] += dV[0] if state_burn[-int(self.dim)] > 0 else -dV[0]
        state_burn[-int(self.dim)+1] += dV[1] if state_burn[-int(self.dim)+1] > 0 else -dV[1]
        if self.dim == 3:
            state_burn[-int(self.dim)+2] += dV[2] if state_burn[-int(self.dim)+2] > 0 else -dV[2]

        solN = self.solver(state_burn, ti, tf, step)
        
        return np.hstack([solution, solN.y[:, 1:]])