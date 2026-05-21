import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.hyp_capture as hp

G = 6.674e-20
mars_mass = 6.39e23
MU = G*mars_mass

mars_SOI = hp.R_soi(228e6, 6.39e23)

R0 = mars_SOI * np.array([np.cos(np.radians(-60)), np.sin(np.radians(-60))])
V0 = np.array([-0.2, 0.8])

dV = hp.dV_hyper(R0, V0, MU)
T = hp.T_hyper(t0=0, r=R0, v=V0, mu=MU)

def main():

    mars = Body(
    position=np.array([0.0, 0.0]),
    velocity=np.array([0.0, 0.0]),
    mass=mars_mass)
    
    spacecraft = Body(
    position=R0,
    velocity=V0,
    mass=1000)

    bodies = [mars, spacecraft]
    imulse0 = Propulsion(tf=T, dVx=dV[0], dVy=dV[1], dVz=0.0)
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=10e6,
        step=10000,
        impulse=[imulse0])

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
