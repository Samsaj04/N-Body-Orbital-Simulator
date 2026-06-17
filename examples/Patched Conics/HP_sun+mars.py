import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
from core.mission_report import MissionReport
import mission_design.hyp_capture as hp
import mission_design.coplanar_maneuvers as md

G = 6.674e-20

sun_mass = 1.9891e30
mars_mass = 6.39e23

MU_sun = G * sun_mass
MU_mars = G * mars_mass

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
# SPACECRAFT INITIAL STATE RELATIVE TO MARS
# =========================================================

mars_SOI = hp.R_soi(R_mars_mag, mars_mass)

R_rel = mars_SOI * np.array([
    np.cos(np.radians(-60)),
    np.sin(np.radians(-60))
])

V_rel = np.array([
    -0.2,
    0.8
])

# =========================================================
# SPACECRAFT HELIOCENTRIC INITIAL STATE
# =========================================================

R_sc = R_mars + R_rel
V_sc = V_mars + V_rel


impulse0, T = md.hyperbolic_capture(
    R_rel,
    V_rel,
    MU_mars
)

def main():

    sun = Body(
        position=np.array([0.0, 0.0]),
        velocity=np.array([0.0, 0.0]),
        mass=sun_mass
    )

    mars = Body(
        position=R_mars,
        velocity=V_mars,
        mass=mars_mass
    )

    spacecraft = Body(
        position=R_sc,
        velocity=V_sc,
        mass=1000
    )

    bodies = [
        sun,
        mars,
        spacecraft
    ]

    controller = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=1e8,
        step=15000,
        impulse=[impulse0]
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