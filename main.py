import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer

m1, m2, m3 = 1.0, 1.0, 1.0
G = 1.0

def main():
    B1 = Body(
        position=np.array([0.97000436, -0.24308753, 0.0]), 
        velocity=np.array([0.466203685, 0.43236573, 0.0]),  
        mass=m1)
    
    B2 = Body(
        position=np.array([-0.97000436, 0.24308753, 0.0]), 
        velocity=np.array([0.466203685, 0.43236573, 0.0]),  
        mass=m2)
    
    B3 = Body(
        position=np.array([0.0, 0.0, 0.0]), 
        velocity=np.array([-0.93240737, -0.86473146, 0.0]),  
        mass=m3)

    impulse0 = Propulsion(tf=5, dVx=-1.0, dVy=-1.0, dVz=1)
    
    bodies = [B1, B2, B3]
    impulses = [impulse0]
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=20,
        fps=120,
        impulse=impulses)

    orbits = controller.run_solution()
    
    viz = Visualizer(
        bodies=bodies,
        trajectories=orbits,
        follow=np.inf,
        speed=10)

    viz.animate()
    
if __name__ == "__main__":
    main()
