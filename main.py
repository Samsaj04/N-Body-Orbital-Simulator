import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer

G = 6.6743e-20          # km^3 / kg / s^2
earth_mass = 5.972e24   # kg
mu = G * earth_mass     # km^3 / s^2

# Orbit Radius
R_earth = 6371      # km
r1 = R_earth + 300  # km
r2 = R_earth + 3300 # km

# Semi-Major Axis for Transfer
aT = (r1 + r2)/2

# Velocity Equations
Vc = lambda rad: np.sqrt(mu / rad)              # Circular Velocity
Ve = lambda rad, a: np.sqrt(mu * (2/rad - 1/a)) # Elliptical Velocity

# Velocity Deltas
V_r1 = Vc(r1)
V_trans1 = Ve(r1, aT)
dV1 = V_trans1 - V_r1

V_r2 = Vc(r2)
V_trans2 = Ve(r2, aT)
dV2 = V_r2 - V_trans2

# Orbital Period
T = lambda a: 2*np.pi*np.sqrt(a**3 / mu)
T0 = T(r1)         # R1 Orbit Period
T_trans = T(aT)    # Transfer Orbit Period
T2 = T(r2)         # R2 Orbit Period

T_tot = (T0 + T_trans/2 + T2)*2

def main():
    earth = Body(
        position=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0]),
        mass=earth_mass
    )

    sat = Body(
        position=np.array([r1, 0.0]),
        velocity=np.array([0.0, V_r1]),
        mass=1
    ) 

    impulse0 = Propulsion(tf=T0, dVx=dV1, dVy=0.0, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_trans/2, dVx=dV2, dVy=0.0, dVz=0.0)
    
    bodies = [earth, sat]
    impulses = [impulse0, impulse1]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T_tot,
        step=20,        # Number of frames per --X-- seconds of simulation.
        impulse=impulses)

    orbits = controller.run_solution()
    viz = Visualizer(
        bodies=bodies,
        trajectories=orbits,
        dim=len(earth.position),
        follow=np.inf,
        speed=5,
        centered=False,
        rel_mass=False
    )

    viz.animate()
    
if __name__ == "__main__":
    main()
