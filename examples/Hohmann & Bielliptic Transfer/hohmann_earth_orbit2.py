import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.orbit_transfers as hm
import mission_design.coplanar_maneuvers as md

G = 6.6743e-20          # km^3 / kg / s^2
earth_mass = 5.972e24   # kg
mu = G * earth_mass     # km^3 / s^2

# Orbit Radius
R_earth = 6371      # km
r1 = R_earth + 300  # km
a1 = 15000          # km

r2 = R_earth + 30000 # km
a2 = 22500      # km

V_r1 = hm.V_orbit(r1, a1, mu)
impulses, T_tot = md.elliptic_hohmann_transfer(r1, a1, r2, a2, mu)

earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
sat = Body(  position=np.array([r1, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1) 

def main():
    
    bodies = [earth, sat]
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot*1.5,
        step=1000,
        impulse=impulses)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=2)

    viz.animate()
    
if __name__ == "__main__":
    main()
