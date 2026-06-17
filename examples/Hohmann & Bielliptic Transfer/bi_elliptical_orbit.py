import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.orbit_transfers as hm
import mission_design.coplanar_maneuvers as md

G = 6.6743e-20          # km^3 / kg / s^2
earth_mass = 5.972e24   # kg
mu = G * earth_mass     # km^3 / s^2

# Orbit Points Distances
R_earth = 6371
Ra = 7000
Rb = 500000
Rc = 105000

# Bielliptic Transfer Data
V_r1 = hm.V_orbit(Ra, Ra, mu)
impulses, T_tot = md.bielliptic_transfer(Ra, Rb, Rc, mu)

earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
sat = Body(  position=np.array([Ra, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1)

def main():
    
    bodies = [earth, sat]
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot*1.5,
        step=10000,
        impulse=impulses)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=15)

    viz.animate()
    
if __name__ == "__main__":
    main()
