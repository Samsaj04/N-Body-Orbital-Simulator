import numpy as np
import matplotlib.pyplot as plt

class MissionReport:
    
    def __init__(self, simulation, orbit):
        
        self.simulation = simulation
        
        self.orbit = orbit
        self.bodies = self.simulation.bodies
        self.steps = len(self.orbit[0][0][0])
        self.G = self.simulation.G
        self.impulse = self.simulation.impulse
        self.dt = self.simulation.tf - self.simulation.ti
        self.times = np.linspace(self.simulation.ti, self.simulation.tf, self.steps)
        
        self.N = len(self.bodies)
        self.dim = self.simulation.dim
        
    #Basic Data
    def _mass(self):
        return np.array([body.mass for body in self.bodies])
    
    def _position(self, step):
        #Get a position vector and magnitude as a function of a step -> |[Rx, Ry, Rz]| for each body
        R_array = []
        for i in range(self.N):
            R = np.array([self.orbit[0][i][j][step] for j in range(self.dim)])
            R_array.append(np.linalg.norm(R))
        return np.array(R_array), np.array([np.linalg.norm(R_array[i]) for i in range(self.N)])
    
    def _velocity(self, step):
        #Get a velocity vector and magnitude as a function of a step -> |[Rx, Ry, Rz]| for each body
        V_array = []
        for i in range(self.N):
            V = np.array([self.orbit[1][i][j][step] for j in range(self.dim)])
            V_array.append(V)
        return V_array, np.array([np.linalg.norm(V_array[i]) for i in range(self.N)])
    
    #Energy
    def kinetic_energy(self, step):
        M = self._mass()
        V = self._velocity(step)[1]
        Ek = 0
        for i in range(self.N):
            Ek += 0.5 * M[i] * V[i]**2
        return Ek
    
    def potential_energy(self, step):
        M = self._mass()
        U = 0.0
        for i in range(self.N):
            for j in range(i+1, self.N):
                Rji = self.relative_state(i, j, step)[0]
                U -= self.G * M[i]*M[j] / (np.linalg.norm(Rji))
        return U
    
    def mechanical_energy(self, step):
        return (self.kinetic_energy(step) + self.potential_energy(step))
    
    # Specific Mechanical Energy
    def epsilon(self):
        MU = self.G*np.sum(self._mass())
        E = []
        for i in range(self.steps):
            Ei = 0.5 * self._velocity(i)[1][-1]**2 - MU/self._position(i)[1][-1]
            E.append(Ei)
        return E
    
    # Energy relative error of mechanical energy throughout the simulation 
    # relative to the initial mechanical energy        
    def energy_error(self):
        E0 = self.mechanical_energy(0)
        error = []
        E_array = []
        for i in range(self.steps):
            Ei = self.mechanical_energy(i)
            epsi = abs(Ei-E0)/abs(E0)
            error.append(epsi)
            E_array.append(Ei)
        return np.array(E_array), np.array(error)
    
    #Minimum distance approach of 2 selected bodies
    def min_approach(self, B1, B2):
        r1 = np.array([self._position(i)[1][B1] for i in range(self.steps)])
        r2 = np.array([self._position(i)[1][B2] for i in range(self.steps)])

        D = r2-r1
        idx = np.argmin(D)
        
        return {
            "distance": D[idx],
            "time": self.times[idx],
            "step": idx
        }
    
    # Velocity and position vector of body B relative to body A   
    def relative_state(self, A, B, step):
        
        r1 = np.array([self.orbit[0][A][i][step] for i in range(self.dim)])
        v1 = np.array([self.orbit[1][A][i][step] for i in range(self.dim)])
        
        r2 = np.array([self.orbit[0][B][i][step] for i in range(self.dim)])
        v2 = np.array([self.orbit[1][B][i][step] for i in range(self.dim)])
        
        return [r2-r1, v2-v1]
    
    # Computes total dV Budget
    def tot_dV(self):
        tot = 0
        for imp in self.impulse:
            dV = np.array([imp.dVx, imp.dVy, imp.dVz])
            tot += np.linalg.norm(dV)
        return tot
    
    def maneuver_summary(self):
        if self.impulse:
            print("\n=== MANEUVERS ===")
            for i, imp in enumerate(self.impulse):
                dV = np.array([imp.dVx, imp.dVy, imp.dVz])
                
                print(f"""
        Burn {i+1}
        Time: {imp.tf:.3f} s
        Delta-Vx: {dV[0]:.6f} km/s
        Delta-Vy: {dV[1]:.6f} km/s
        Delta-Vz: {dV[2]:.6f} km/s
    
        Delta-V_tot: {np.linalg.norm(dV):.6f} km/s
                """)
            else:
                pass
    
    def plot_epsilon(self):
        if self.simulation.N == 2:
            plt.plot(self.times, self.epsilon())
        
            plt.title("Specific Mechanical Energy vs Time (s)")
            plt.xlabel("Time (s)")
            plt.ylabel("Specific Mechanical Energy")
            
            plt.grid(True)
            plt.show()
        else:
            print("Specific Mechanical Energy can only be computed and ploted with a 2 Body Simulation")

                
    def plot_energy(self):
        E_arr, error = self.energy_error()
        kin = [self.kinetic_energy(i) for i in range(self.steps)]
        pot = [self.potential_energy(i) for i in range(self.steps)]
        
        plt.plot(self.times, kin, label="Kinetic Energy")
        plt.plot(self.times, pot, label="Potential Energy")
        plt.plot(self.times, E_arr, label="Mechanical Energy")
        plt.plot(self.times, error, label="Relative Energy Error")
        
        plt.title("Mechanical Energy vs Time (s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Mechanical Energy")
        
        plt.legend()
        plt.grid(True)
        plt.show()
        
    #Summary
    def mission_log(self, plot=True):
        E0 = self.mechanical_energy(0)
        Ef = self.mechanical_energy(-1)
        print("\n======== MISSION SUMMARY ========")
        
        print(f"""
    Bodies: {self.N}
    Simulation Time: {self.times[0]} -> {self.times[-1]} s
    Total Delta-V: {self.tot_dV():.6f} km/s
    Initial Energy: {E0:.6e}
    Final Energy: {Ef:.6e}
    Relative Energy Error: {abs(Ef-E0)/abs(E0) * 100:.3e}%
    Minimum Approach of Bodies {self.N-1} and {self.N}:
        Distance: {self.min_approach(-2, -1)["distance"]:.4f} km
        Time: {self.min_approach(-2, -1)["time"]:.3f} s
        Step: {self.min_approach(-2, -1)["step"]}
            """)
        print("====================================")
        self.maneuver_summary()
        
        #plots
        if plot:
            self.plot_energy()
            self.plot_epsilon()

    # Plots the relative distance between the spacecraft and a selected Body through time
    def plot_rel_distance(self, idx_sc, idx_target):

        Pi_sol = self.orbit[0]
        
        pos_target = Pi_sol[idx_target]
        pos_sc = Pi_sol[idx_sc]
        
        rel_vec = pos_sc - pos_target
        dist = np.linalg.norm(rel_vec, axis=0)
        
        tot_steps = len(dist)
        times = np.linspace(self.simulation.ti, self.simulation.tf, tot_steps)
        
        # Convert seconds to days
        time_days = times / 86400
        
        plt.figure(figsize=(10, 6))
        plt.plot(time_days, dist, label=f'Distance Spacecraft - Body {idx_target+1}', linewidth=2)
        
        final_rad = dist[-1]
        plt.axhline(y=final_rad, color='red', linestyle='--', alpha=0.7, 
                    label=f'Final Radius: {final_rad:.2f} km')
        
        plt.title('Relative Distance Throughout time', fontsize=14, fontweight='bold')
        plt.xlabel('Simulation Time (Days)', fontsize=12)
        plt.ylabel('Distance (km)', fontsize=12)
        
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend(fontsize=12)
        plt.tight_layout()
        plt.show()