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
r2 = R_earth + 4000 # km

# Hohmann Transfer Data
V_r1 = hm.V_orbit(r1, r1, mu)
dV1, dV2, T_trans = hm.hohmann_transfer(r1, r2, mu)
T0, T2 = hm.T_imp(r1, r2, mu)  # R1 & R2 Orbit Period

T_tot = T0 + T_trans/2 + T2

def main():
    earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]),  mass=earth_mass)
    sat = Body(  position=np.array([r1, 0.0]),  velocity=np.array([0.0, V_r1]), mass=1) 

    impulse0 = Propulsion(tf=T0,             dVx=dV1, dVy=0.0, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_trans/2, dVx=dV2, dVy=0.0, dVz=0.0)
    
    bodies = [earth, sat]
    impulses = [impulse0, impulse1]
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot*1.5,
        step=20,        # Number of frames per --X-- seconds of simulation.
        impulse=impulses)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        trajectories=orbits,
        follow=np.inf,
        speed=5
    )

    viz.animate()
    
if __name__ == "__main__":
    main()
