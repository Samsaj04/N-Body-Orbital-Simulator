import numpy as np
from .physics_engine import PhysicsEngine

class SimulationController:
    def __init__(self, bodies, G, ti, tf, fps=120, impulse = []):
        self.bodies = bodies
        self.G = G
        self.ti = ti
        self.tf = tf
        self.fps = fps
        self.impulse = impulse
        
        self.N = len(self.bodies)
        self.dim = len(self.bodies[0].position)
        masses = [b.mass for b in bodies]
        
        self.engine = PhysicsEngine(self.bodies, self.G, masses, self.dim)
        self.initial_conditions = np.array([
            np.array([b.position for b in self.bodies]),
            np.array([b.velocity for b in self.bodies])
        ]).ravel()
        
    def num_steps_(self, ti, tf):
        return int((tf-ti)*self.fps)
        
    def run_solution(self):
        ft = 1
        if self.num_steps_(self.ti, self.tf) > 10000:
            ft = self.num_steps_(self.ti, self.tf)/10000
        
        if not self.impulse:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.tf, int(self.num_steps_(self.ti, self.tf)/ft))
            y_tot = sol0.y
        else:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.impulse[0].tf, int(self.num_steps_(self.ti, self.impulse[0].tf)/ft))
            y_tot = sol0.y
            
            imp_tf = np.concat(([pulse.tf for pulse in self.impulse], [self.tf]), axis=0)

            for pulse in range(len(self.impulse)):
                dV = [self.impulse[pulse].dVx, self.impulse[pulse].dVy, self.impulse[pulse].dVz]
                sol_n = self.engine.add_impulse_(y_tot, imp_tf[pulse], imp_tf[pulse+1], dV, int(self.num_steps_(imp_tf[pulse], imp_tf[pulse+1])/ft))
                y_tot = sol_n

        Pi_sol = []
        for i in range(self.N):
            body_coords = y_tot[i*self.dim : i*self.dim + self.dim]
            Pi_sol.append(body_coords)
            
        return Pi_sol