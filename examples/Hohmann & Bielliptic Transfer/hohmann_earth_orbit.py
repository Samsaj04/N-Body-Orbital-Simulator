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
r2 = R_earth + 8000 # km

V_r1 = hm.V_orbit(r1, r1, mu)
impulse, T_tot = md.hohmann_transfer(r1, r2, mu)

def main():
    earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
    sat = Body(  position=np.array([r1, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1)
    
    bodies = [earth, sat]
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot*1.5,
        step=10000,
        impulse=impulse)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=15)

    viz.animate()
    
if __name__ == "__main__":
    main()
