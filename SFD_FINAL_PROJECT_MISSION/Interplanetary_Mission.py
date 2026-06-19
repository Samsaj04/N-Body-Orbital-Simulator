import numpy as np
from core.entities import Body, Propulsion
from core.simulation_controller import SimulationController
from core.visualizer import Visualizer
from core.mission_report import MissionReport
import mission_design.hyp_capture as hp

# CONSTANTS
G = 6.674e-20
sun_mass = 1.9891e30
earth_mass = 5.972e24
mars_mass = 6.39e23

MU_sun = G * sun_mass
MU_earth = G * earth_mass
MU_mars = G * mars_mass

R_earth_mag = 149.6e6 # Distance from Earth to the sun
R_mars_mag = 228.0e6  # Distance from Mars to the sun
R_mars_surface = 3389.5 # Mars Radius

V_earth_mag = np.sqrt(MU_sun / R_earth_mag) # Earth's Orbital Velocity
V_mars_mag = np.sqrt(MU_sun / R_mars_mag)   # mars' Orbital Velocity

SOI_mars = hp.R_soi(R_mars_mag, mars_mass) #Sphere of Influence of Mars

# =========================================================
# TRANS MARS INJECTION
r_pe = 6371.0 + 400.0  # LEO Orbit
v_leo = np.sqrt(MU_earth / r_pe) # SC LEO Velocity
T_leo = 2 * np.pi * np.sqrt(r_pe**3 / MU_earth) # Period of SC LEO Orbit

# Modified Hohmann Transfer
a_trans = (R_earth_mag + R_mars_mag) / 2 # Semi Major Axis of Transfer orbit from Earth to Mars
V_peri = np.sqrt(MU_sun * (2/R_earth_mag - 1/a_trans)) # Velocity at Periapsis (In Earth)
V_apo = np.sqrt(MU_sun * (2/R_mars_mag - 1/a_trans))   # Velocity at Apoapsis (In Mars)
TOF_hohmann = np.pi * np.sqrt((a_trans**3) / MU_sun) # Time of Flight of Transfer Orbit

# Hyperbolic Excess Velocity for Hohmann Transfer from Earth to Mars
juan_tmi = hp.hyp_excess_speed(r_pe, R_earth_mag, R_mars_mag, MU_earth)

# =========================================================
# ORBITS ORIENTATION
omega_earth = np.sqrt(MU_sun / R_earth_mag**3) # Earth's Angular Velocity
omega_mars = np.sqrt(MU_sun / R_mars_mag**3)   # Mars' Angular Velocity

# Compute Theta0 in order for the mission to complete an initial
# revolution of the LEO orbit before departure
theta_e0 = -omega_earth * T_leo
theta_m0 = np.pi - omega_mars * (TOF_hohmann - T_leo)

v_inf_req_earth = V_peri - V_earth_mag # Spacecraft's Hyperbolic Excess Velocity at Earth's SOI
e_hyp = 1 + (r_pe * v_inf_req_earth**2) / MU_earth # Eccentricity of Hyperbolic Orbit after Burn
nu_inf = np.arccos(-1 / e_hyp) # True Anomaly of the Hyperbolic orbit
                               # (Angle between the line of periapsis 
                               # and the asymptote of the hyperbola)

theta_vel_earth = theta_e0 + (np.pi / 2)
theta_periapsis = theta_vel_earth - nu_inf # Calculates the required periapsis angle to align the 
                                           # hyperbolic escape asymptote parallel to Earth's velocity vector.

# =========================================================
# INITIAL STATE VECTORS (T=0)
R_earth = np.array([R_earth_mag * np.cos(theta_e0), R_earth_mag * np.sin(theta_e0)])
V_earth = np.array([-V_earth_mag * np.sin(theta_e0), V_earth_mag * np.cos(theta_e0)])

R_mars = np.array([R_mars_mag * np.cos(theta_m0), R_mars_mag * np.sin(theta_m0)])
V_mars = np.array([-V_mars_mag * np.sin(theta_m0), V_mars_mag * np.cos(theta_m0)])

dir_pos = np.array([np.cos(theta_periapsis), np.sin(theta_periapsis)]) #Unitary Vector of SC Position
R_sc = R_earth + (dir_pos * r_pe) # Spacecraft's Position Relative to the Sun

dir_v = np.array([-np.sin(theta_periapsis), np.cos(theta_periapsis)]) #Unitary Vector of SC Velocity
V_sc = V_earth + (dir_v * v_leo) # Spacecraft's Velocity Relative to the Sun



def main():
    sun = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]), mass=sun_mass)
    earth = Body(position=R_earth, velocity=V_earth, mass=earth_mass)
    mars = Body(position=R_mars, velocity=V_mars, mass=mars_mass)
    spacecraft = Body(position=R_sc, velocity=V_sc, mass=1000)

    bodies = [sun, earth, mars, spacecraft]

    # FIRST EXECUTION =================================
    # To find Periapsis at Mars
    t_final = T_leo + TOF_hohmann + (300*24*3600) # Time until intercept + 300 days
    impulse_tmi = Propulsion(tf=T_leo, dVx=dir_v[0]*juan_tmi, dVy=dir_v[1]*juan_tmi, dVz=0.0)

    controller0 = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=t_final,
        step=20000,
        impulse=[impulse_tmi] 
    )

    orbits = controller0.run_solution()

    t_SOI = controller0.find_time_pos(orbits, 3, 2, SOI_mars)
    
    # Time of Periapsis, Velocity change
    t_burn, dV_vec = controller0.find_periapsis(orbits, 3, 2, MU_mars) 
      
    # SECOND EXECUTION =================================
    # Executing dV at Mars' Periapsis
    impulse_tmi = Propulsion(tf=T_leo, dVx=dir_v[0] * juan_tmi, dVy=dir_v[1] * juan_tmi, dVz=0.0)
    impulse_hyp = Propulsion(tf=t_burn, dVx=dV_vec[0], dVy=dV_vec[1], dVz=0.0)

    t_final_real = t_burn + (70*24*3600) # Time until intercept + 70 days
    controller_real = SimulationController(
        bodies=bodies,
        G=G,
        ti=0,
        tf=t_final_real,
        step=20000,
        impulse=[impulse_tmi, impulse_hyp] )

    orbits = controller_real.run_solution()

    viz = Visualizer(
        bodies=bodies,
        orbit=orbits,
        speed=50)
    
    report = MissionReport(simulation=controller_real, orbit=orbits)
    
    viz.animate()
    report.mission_log(True)

if __name__ == "__main__":
    main()