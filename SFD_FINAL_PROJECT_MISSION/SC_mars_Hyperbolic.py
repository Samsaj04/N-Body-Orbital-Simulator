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

# =========================================================
# Plots the relative distance between SC and Mars to confirm orbital circularization
def plot_distance(orbits, ti, tf):
    
    pos_mars = orbits[0][0] 
    pos_sc = orbits[0][1]   
    
    # Relative distance vectors
    rel_vec = pos_sc - pos_mars
    dist = np.linalg.norm(rel_vec, axis=0)
    
    # Time axis in days
    time_days = np.linspace(ti, tf, len(dist))/86400
    burn_day = t_burn/86400
    
    plt.figure(figsize=(10, 6))
    plt.plot(time_days, dist, label='Distance Spacecraft - Mars', color='#2ca02c', linewidth=2)
    plt.axvline(x=burn_day, color='red', linestyle='--', label=f'Burn at Periapsis (Day {burn_day:.2f})')
    
    # Circular parking orbit radius
    final_radius = dist[-1]
    plt.axhline(y=final_radius, color='blue', linestyle=':', label=f'Final Radius: {final_radius:.2f} km')
    
    plt.title('Hyperbolic Capture')
    plt.xlabel('Time (Days)')
    plt.ylabel('Distance (km)')
    plt.grid(True, alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()

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
    plot_distance(orbits, 0, t_final)

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