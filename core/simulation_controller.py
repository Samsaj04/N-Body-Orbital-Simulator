import numpy as np
from .physics_engine import PhysicsEngine

class SimulationController:
    def __init__(self, bodies, G, ti, tf, step, impulse = []):
        self.bodies = bodies
        self.G = G
        self.ti = ti
        self.tf = tf
        self.step = step
        self.impulse = impulse
        
        self.N = len(self.bodies)
        self.dim = len(self.bodies[0].position)
        masses = [b.mass for b in bodies]
        
        self.engine = PhysicsEngine(self.bodies, self.G, masses, self.dim)
        self.initial_conditions = np.array([
            np.array([b.position for b in self.bodies]),
            np.array([b.velocity for b in self.bodies])
        ]).ravel()
        
    def run_solution(self):
        
        if not self.impulse:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.tf, self.step)
            y_tot = sol0.y
        else:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.impulse[0].tf, self.step)
            y_tot = sol0.y
            
            imp_tf = np.concat(([pulse.tf for pulse in self.impulse], [self.tf]), axis=0)

            for pulse in range(len(self.impulse)):
                dV = [self.impulse[pulse].dVx, self.impulse[pulse].dVy, self.impulse[pulse].dVz]
                sol_n = self.engine.add_impulse_(y_tot, imp_tf[pulse], imp_tf[pulse+1], dV, self.step)
                y_tot = sol_n

            
        Pi_sol = []
        for i in range(self.N):
            body_coords = y_tot[i*self.dim : i*self.dim + self.dim]
            Pi_sol.append(body_coords)
            
        return Pi_sol