import numpy as np
from scipy.integrate import solve_ivp

class PhysicsEngine:
    def __init__(self, bodies, G, masses, dim):
        self.G = G
        self.bodies = bodies
        self.masses = masses
        self.dim = dim
        self.N = len(masses)

    def Fij(self, mi, mj, ri, rj):
        return self.G * (mi*mj)/(np.linalg.norm(rj - ri)**3) * (rj - ri)
    
    def accel(self, t, S):
        
        Pitt = np.zeros((self.N, self.dim))
        S_og = S.reshape((2, self.N, self.dim))
        
        pos = S_og[0]
        vel = S_og[1]

        for i in range(self.N):
            mi = self.masses[i]
            pi = pos[i]
            for j in range(self.N):
                if j != i:
                    Pitt[i] += (self.Fij(mi, self.masses[j], pi, pos[j]))
                else:
                    continue
            Pitt[i] /= self.masses[i]
            
        return np.array([vel, Pitt]).ravel()
    
    def solver(self, state, ti, tf, step):
        
        sol0 = solve_ivp(
            fun=self.accel,
            t_span=(ti, tf),
            y0=state,
            t_eval=np.linspace(ti, tf, step),
            method='DOP853',
            rtol=1e-10,
            atol=1e-10,
            dense_output=True)
        return sol0

    def add_impulse_(self, solution, ti, tf, dV, step):
        
        state_burn = solution[:, -1].copy()
        state_burn[-self.dim] += dV[0] if state_burn[-self.dim] > 0 else -dV[0]
        state_burn[-self.dim+1] += dV[1] if state_burn[-self.dim+1] > 0 else -dV[1]
        state_burn[-self.dim+2] += dV[2] if state_burn[-self.dim+2] > 0 else -dV[2]

        solN = self.solver(state_burn, ti, tf, step)
        
        return np.hstack([solution, solN.y[:, 1:]])