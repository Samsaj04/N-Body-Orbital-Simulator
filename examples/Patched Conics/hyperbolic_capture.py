import numpy as np
from core.entities import Body
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.hyp_capture as hp
import mission_design.coplanar_maneuvers as md

G = 6.674e-20
mars_mass = 6.39e23
MU = G*mars_mass

mars_SOI = hp.R_soi(228e6, 6.39e23)

R0 = mars_SOI * np.array([np.cos(np.radians(-60)), np.sin(np.radians(-60))])
V0 = np.array([-0.2, 0.8])

impulse0, T = md.hyperbolic_capture(R0, V0, MU)

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
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=10e6,
        step=10000,
        impulse=[impulse0])

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=20)

    viz.animate()
    
if __name__ == "__main__":
   main()
