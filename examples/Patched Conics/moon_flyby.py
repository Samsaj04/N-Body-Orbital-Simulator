import numpy as np
from core.entities import Body
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer

G = 6.6743e-20  # km^3 / kg / s^2

def main():

    moon = Body(
    position=np.array([384400.0, 0.0]),
    velocity=np.array([0.0, 1.01]),
    mass=7.348e22
)

    earth = Body(
    position=np.array([0.0, 0.0]),
    velocity=np.array([-0.0124, 0.0]),
    mass=5.972e24
)

    spacecraft = Body(
    position=np.array([6671.0, 0.0]),
    velocity=np.array([7.77922192, 7.55017334]),
    mass=1000)

    bodies = [earth, moon, spacecraft]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=2.3606e6/4*1.5,
        step=10000,
        impulse=[])

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=35)

    viz.animate()

if __name__ == "__main__":
    main()