import numpy as np
import matplotlib.pyplot as plt
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
from core.mission_report import MissionReport

# =========================================================
# CONSTANTS
G = 6.674e-20
mars_mass = 6.39e23

# =========================================================
# INITIAL STATE VECTORS - Took from Telemetry of the Interplantary Mission
# The spacecraft begins at the edge of Mars' Sphere of Influence
R_sc_soi = np.array([-364996.52939287, -447431.67934800])
V_sc_soi = np.array([-0.15016523, 2.71922313])

# =========================================================
# MARS ORBIT INSERTION MANEUVER
t_burn = 21726037.0285 - 21569074.4402 # seconds
dV_moi = np.array([-0.10738625, -2.40302961])

# Impulse for circularization
impulse_moi = Propulsion(tf=t_burn, dVx=dV_moi[0], dVy=dV_moi[1], dVz=0.0)

def main():

    mars = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]), mass=mars_mass)
    spacecraft = Body(position=R_sc_soi, velocity=V_sc_soi, mass=1000.0)

    bodies = [mars, spacecraft]

    # Simulate until burn + 150 Days
    t_final = t_burn + 150*24*3600
    
    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=t_final,
        step=20000, 
        impulse=[impulse_moi])

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=30)

    report = MissionReport(simulation=controller, orbit=orbits)

    viz.animate()
    report.mission_log(True)

if __name__ == "__main__":
    main()