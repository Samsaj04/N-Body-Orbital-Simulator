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
        
    def num_steps_(self, ti, tf):
        return int(np.ceil((tf-ti)/(self.tf/self.step)))
        
    def run_solution(self):        
        if not self.impulse:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.tf, self.num_steps_(self.ti, self.tf))
            y_tot = sol0.y
        else:
            sol0 = self.engine.solver(self.initial_conditions, self.ti, self.impulse[0].tf, self.num_steps_(self.ti, self.impulse[0].tf))
            y_tot = sol0.y
            imp_tf = np.concat(([pulse.tf for pulse in self.impulse], [self.tf]), axis=0)

            for pulse in range(len(self.impulse)):
                dV = [self.impulse[pulse].dVx, self.impulse[pulse].dVy, self.impulse[pulse].dVz]
                sol_n = self.engine.add_impulse_(y_tot, imp_tf[pulse], imp_tf[pulse+1], dV, self.num_steps_(imp_tf[pulse], imp_tf[pulse+1]))
                y_tot = sol_n
                
        Pi_sol = []
        Vi_sol = []
        offs = self.N*self.dim
        
        for i in range(self.N):
            body_coords = y_tot[i*self.dim : self.dim*(i+1)]
            Pi_sol.append(body_coords)
            
            body_speeds = y_tot[i*self.dim + offs : self.dim*(i+1) + offs]
            Vi_sol.append(body_speeds)
        self.y_tot = y_tot
        return Pi_sol, Vi_sol
    
    # Find time when a certain relative position between the Spacecraft and a target is reached
    def find_time_pos(self, orbits, idx_sc, idx_target, target_pos):
    
        Pi_sol = orbits[0]
        Vi_sol = orbits[1]
        
        pos_target = Pi_sol[idx_target]
        pos_sc = Pi_sol[idx_sc]
        
        vel_target = Vi_sol[idx_target]
        vel_sc = Vi_sol[idx_sc]
        
        rel_vec = pos_sc - pos_target
        dist = np.linalg.norm(rel_vec, axis=0)
        
        idx_match = np.argmin(np.abs(dist - target_pos))
        
        t_burn = self.ti + (idx_match / (self.step-1)) * (self.tf - self.ti)
        
        rel_R = rel_vec[:, idx_match]
        rel_V = vel_sc[:, idx_match] - vel_target[:, idx_match]
        
        #LOG
        print("POSITION FINDER")
        print("-"*40)
        print(f"Target Position:            {target_pos:.4f} km")
        print(f"Step index of Position:     {idx_match}")
        print(f"Time of Burn:               {t_burn:.4f} s")
        print(f"Magnitude of Relative pos.: {dist[idx_match]:.4f} km")
        print(f"Position Vector:            {rel_R}")
        print(f"Rel Velocity Magnitude:     {np.linalg.norm(rel_V):.4f} km/s")
        print(f"Velocity Vector:            {rel_V}")
        print("\n")
        
        return t_burn
    
    # Find time when the periapsis of a 2 Body problem is reached
    def find_periapsis(self, orbits, idx_sc, idx_target, MU_target):
            Pi_sol = orbits[0]
            Vi_sol = orbits[1]
            
            pos_target = Pi_sol[idx_target]
            pos_sc = Pi_sol[idx_sc]
            
            vel_target = Vi_sol[idx_target]
            vel_sc = Vi_sol[idx_sc]
            
            rel_vec = pos_sc - pos_target
            dist = np.linalg.norm(rel_vec, axis=0)
            
            idx_match = np.argmin(dist)
            r_min = dist[idx_match]
            t_burn = self.ti + (idx_match / (self.step - 1)) * (self.tf - self.ti)
            
            rel_V = vel_sc[:, idx_match] - vel_target[:, idx_match]
            v_current_mag = np.linalg.norm(rel_V)
            
            Vc_mag = np.sqrt(MU_target / r_min)
            rel_V_new = rel_V * (Vc_mag / v_current_mag)
            
            dV_vec_normal = rel_V_new - rel_V
            
            abs_V_sc = vel_sc[:, idx_match] 
            dV_vec_final = dV_vec_normal * np.sign(abs_V_sc)
            
            #LOG
            print("PERIAPSIS FINDER")
            print("-"*40)
            print(f"Step Index of Periapsis:    {idx_match}")
            print(f"Time of Burn:               {t_burn:.4f} s")
            print(f"Periapsis Radius:           {r_min:.4f} km")
            print(f"Velocity at Periapsis:      {v_current_mag:.4f} km/s")
            print(f"Velocity Vector:            {rel_V} km/s")
            print(f"Required Circular Velocity: {Vc_mag:.4f} km/s")
            print(f"Final Delta of Velocity:    {dV_vec_final}")
            print("\n")
            
            return t_burn, dV_vec_final