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
        #Get a position magnitude as a function of a step -> |[Rx, Ry, Rz]| for each body
        R_array = []
        for i in range(self.N):
            R = np.array([self.orbit[0][i][j][step] for j in range(self.dim)])
            R_array.append(np.linalg.norm(R))
        return np.array(R_array), np.array([np.linalg.norm(R_array[i]) for i in range(self.N)])
    
    def _velocity(self, step):
        V_array = []
        for i in range(self.N):
            V = np.array([self.orbit[1][i][j][step] for j in range(self.dim)])
            V_array.append(V)
        return V_array, np.array([np.linalg.norm(V_array[i]) for i in range(self.N)])
    
    #Momentum
    def linear_momentum(self, step):
        M = self._mass()
        V = self._velocity(step)[1]
        return np.sum(M[:, None] * V, axis=0)
    
    def angular_momentum(self, step):
        M = self._mass()
        V = self._velocity(step)[1]
        R = self._position(step)[1]
        return np.sum(np.cross(R, M[:, None] * V), axis=0)
    
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
    
    def epsilon(self):
        MU = self.G*np.sum(self._mass())
        E = []
        for i in range(self.steps):
            Ei = 0.5 * self._velocity(i)[1][-1]**2 - MU/self._position(i)[1][-1]
            E.append(Ei)
        return E
        
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
    
    #Orbital Analysis
    def min_approach(self, B1, B2):
        r1 = np.array([self._position(i)[1][B1] for i in range(self.steps)])
        r2 = np.array([self._position(i)[1][B2] for i in range(self.steps)])
        #r1 = self.orbit[0][B1]
        #r2 = self.orbit[0][B2]
        D = r2-r1
        idx = np.argmin(D)
        
        return {
            "distance": D[idx],
            "time": self.times[idx],
            "step": idx
        }
        
    def relative_state(self, A, B, step):
        
        r1 = np.array([self.orbit[0][A][i][step] for i in range(self.dim)])
        v1 = np.array([self.orbit[0][A][i][step] for i in range(self.dim)])
        
        r2 = np.array([self.orbit[0][B][i][step] for i in range(self.dim)])
        v2 = np.array([self.orbit[0][B][i][step] for i in range(self.dim)])
        
        return (r2-r1, v2-v1)
    
    #Impulses
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
        Delta-V: {np.linalg.norm(dV):.6f} km/s
                """)
            else:
                pass
    
    def plot_epsilon(self):
        plt.plot(self.times, self.epsilon())
        
        plt.title("Specific Mechanical Energy vs Time (s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Specific Mechanical Energy")
        
        plt.grid(True)
        plt.show()
                
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
    Minimum Approach:
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
            
        