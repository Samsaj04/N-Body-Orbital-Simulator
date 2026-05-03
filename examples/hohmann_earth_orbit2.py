import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.hohmann as hm

G = 6.6743e-20          # km^3 / kg / s^2
earth_mass = 5.972e24   # kg
mu = G * earth_mass     # km^3 / s^2

# Orbit Radius
R_earth = 6371      # km
r1 = R_earth + 300  # km
a1 = 15000          # km

r2 = R_earth + 30000 # km
a2 = 22500      # km


# Hohmann Transfer Data
V_r1 = hm.V_orbit(r1, a1, mu)
dV1, dV2, T_trans = hm.elliptic_HT(r1, a1, r2, a2, mu)
T0, T2 = hm.T_imp(a1, a2, mu)  # R1 & R2 Orbit Period

T_tot = T0 + T_trans/2 + T2

def main():
    earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
    sat = Body(  position=np.array([r1, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1) 

    impulse0 = Propulsion(tf=T0,             dVx=0.0, dVy=dV1, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_trans/2, dVx=0.0, dVy=dV2, dVz=0.0)
    
    bodies = [earth, sat]
    impulses = [impulse0, impulse1]
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot*1.5,
        impulse=impulses)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        trajectories=orbits,
        follow=np.inf,
        speed=20
    )

    viz.animate()
    
if __name__ == "__main__":
    main()
