import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.coplanar_maneuvers as hm

G = 6.6743e-20          # km^3 / kg / s^2
earth_mass = 5.972e24   # kg
mu = G * earth_mass     # km^3 / s^2

# Orbit Points Distances
R_earth = 6371
Ra = 7000
Rb = 500000
Rc = 105000

# Hohmann Transfer Data
V_r1 = hm.V_orbit(Ra, Ra, mu)
dV1, dV2, T_t1, T_t2 = hm.bi_elliptic(Ra, Rb, Rc, mu)
T0, T2 = hm.T_biell(Ra, Rb, Rc, mu)
T_tot = T0 + T_t1/2 + T_t2/2 + T2

def main():
    earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
    sat = Body(  position=np.array([Ra, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1) 

    impulse0 = Propulsion(tf=T0,          dVx=0.0, dVy=dV1, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_t1/2, dVx=0.0, dVy=dV2, dVz=0.0)
    
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
        speed=5)

    viz.animate()
    
if __name__ == "__main__":
    main()
