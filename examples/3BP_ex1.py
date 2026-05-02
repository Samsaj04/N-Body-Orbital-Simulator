import numpy as np
from core.entities import Body
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer

G = 6.6743e-20  # km^3 / kg / s^2

def main():
    
    m1 = 5e24
    m2 = 4e24
    m3 = 3e24

    b1 = Body(
        position=np.array([-15000.0, 0.0]),
        velocity=np.array([0.0, -1.5]),
        mass=m1
    )

    b2 = Body(
        position=np.array([15000.0, 0.0]),
        velocity=np.array([0.0, 1.2]),
        mass=m2
    )

    b3 = Body(
        position=np.array([0.0, 20000.0]),
        velocity=np.array([-1.0, 0.0]),
        mass=m3
    )

    bodies = [b1, b2, b3]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=1e6/2,
        impulse=[] 
    )

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        trajectories=orbits,
        follow=350,
        speed=10
    )

    viz.animate()

if __name__ == "__main__":
    main()