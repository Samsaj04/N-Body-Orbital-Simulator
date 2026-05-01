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
        
        num_steps = max(int(abs(tf-ti) / step), 2)
        sol0 = solve_ivp(
            fun=self.accel,
            t_span=(ti, tf),
            y0=state,
            t_eval=np.linspace(ti, tf, num_steps),
            method='DOP853',
            rtol=1e-10,
            atol=1e-10,
            dense_output=True)
        return sol0

    def add_impulse_(self, solution, ti, tf, dV, step):
        
        state_burn = solution[:, -1].copy()
        
        r = state_burn[:self.dim]
        v = state_burn[-self.dim:]

        r_hat = r / np.linalg.norm(r)
        v_hat = v / np.linalg.norm(v)
        
        r_3D = np.append(r, 0) if self.dim == 2 else r
        v_3D = np.append(v, 0) if self.dim == 2 else r
        
        h_3D = np.cross(r_3D, v_3D)
        n_hat_3D = h_3D / np.linalg.norm(h_3D)
        n_hat = n_hat_3D[:self.dim]

        dV_t = dV[0]
        dV_r = dV[1]
        dV_n = dV[2]

        state_burn[-self.dim:] += (dV_t * v_hat) + (dV_r * r_hat) + (dV_n * n_hat)

        solN = self.solver(state_burn, ti, tf, step)
        
        return np.hstack([solution, solN.y[:, 1:]])