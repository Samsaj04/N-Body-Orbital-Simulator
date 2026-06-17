import numpy as np
from core.entities import Body
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer

G = 6.6743e-20  # km^3 / kg / s^2
m1, m2, m3 = 5e24, 4e24, 3e24

def main():
    
    b1 = Body(
        position=np.array([-15000.0, 0.0]),
        velocity=np.array([0.0, -1.5]),
        mass=m1)

    b2 = Body(
        position=np.array([15000.0, 0.0]),
        velocity=np.array([0.0, 1.2]),
        mass=m2)

    b3 = Body(
        position=np.array([0.0, 20000.0]),
        velocity=np.array([-1.0, 0.0]),
        mass=m3)

    bodies = [b1, b2, b3]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=1e6/3,
        step=10000)

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=350,
        speed=10)

    viz.animate()

if __name__ == "__main__":
    main()