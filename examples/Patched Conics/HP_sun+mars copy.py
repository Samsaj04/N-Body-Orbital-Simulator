import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
import mission_design.orbit_transfers as ot
import mission_design.coplanar_maneuvers as cp

# =========================================================
# CONSTANTS
# =========================================================

G = 6.674e-20  # km^3/kg/s^2

sun_mass = 1.9891e30
earth_mass = 5.972e24
mars_mass = 6.39e23

MU_sun = G * sun_mass
MU_earth = G * earth_mass

# =========================================================
# SUN
# =========================================================

R_sun = np.array([0.0, 0.0])
V_sun = np.array([0.0, 0.0])

# =========================================================
# EARTH HELIOCENTRIC ORBIT
# =========================================================

R_earth_mag = 149.6e6  # km
V_earth_mag = np.sqrt(MU_sun / R_earth_mag)

R_earth = np.array([
    R_earth_mag,
    0.0
])

V_earth = np.array([
    0.0,
    V_earth_mag
])

# =========================================================
# MARS HELIOCENTRIC ORBIT
# =========================================================

R_mars_mag = 228e6  # km
V_mars_mag = np.sqrt(MU_sun / R_mars_mag)

R_mars = np.array([
    R_mars_mag,
    0.0
])

V_mars = np.array([
    0.0,
    V_mars_mag
])

# =========================================================
# SPACECRAFT IN LEO
# =========================================================

R_earth_radius = 6378.0  # km
altitude = 400.0         # km

R_leo = R_earth_radius + altitude

V_leo = np.sqrt(MU_earth / R_leo)

# posición relativa respecto a la Tierra
R_rel = np.array([
    R_leo,
    0.0
])

# velocidad tangencial relativa respecto a la Tierra
V_rel = np.array([
    0.0,
    V_leo
])

# estado heliocéntrico de la nave
R_sc = R_earth + R_rel
V_sc = V_earth# + V_rel

impulses, T = cp.hohmann_transfer(R_earth_mag, R_mars_mag, MU_sun)

#V1 = lp.lambert([R_sc[0], R_sc[1], 0.0], [149847352.34166905, 149847352.34166905, 0.0], 6228913.748, MU_sun)
#print(V1)
# =========================================================
# MAIN
# =========================================================

def main():

    sun = Body(
        position=R_sun,
        velocity=V_sun,
        mass=sun_mass
    )

    earth = Body(
        position=R_earth,
        velocity=V_earth,
        mass=earth_mass
    )

    mars = Body(
        position=R_mars,
        velocity=V_mars,
        mass=mars_mass
    )

    spacecraft = Body(
        position=R_sc,
        velocity=V_sc,
        mass=1000.0
    )

    bodies = [sun,  mars, spacecraft]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=T*2,
        step=1000,
        impulse=impulses
    )

    orbits = controller.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        follow=np.inf,
        speed=1
    )

    viz.animate()

if __name__ == "__main__":
    main()