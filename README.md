# N Body Orbital Simulator
A high-precision N-body orbital dynamics simulator in Python, featuring impulsive maneuver planning and 2D/3D visualizations.

## Features
* **High-Precision Integration:** Uses SciPy's DOP853 solver (an 8th-order Runge-Kutta method) for accurate orbital propagation over long periods.
* **Impulsive Maneuvers ($\Delta V$):** Program burns in Prograde/Retrograde, Radial, and Normal directions at specific timestamps to simulate orbital transfers.
* **N-Body Physics:** Capable of simulating interactions between any number of celestial bodies and spacecraft.
* **Dynamic Visualization:** 2D and 3D animated plots using Matplotlib, with options for relative mass scaling, adjustable playback speed, and a camera that can follow the system's center of mass.
* **Modular Architecture:** Clean, object-oriented design separating physics, control, and visualization.

## Project Structure
```text
orbital_simulator/
│
├── core/
│   ├── __init__.py              # Package initializer to simplify imports.
│   ├── entities.py              # Data structures for celestial bodies and propulsion parameters.
│   ├── physics_engine.py        # Core orbital mechanics: gravity equations and ODE integration.
│   ├── simulation_controller.py # Logic for time-stepping, burn scheduling, and state management.
│   └── visualizer.py            # Rendering engine for 2D/3D Matplotlib animations and plots.
│
├── main.py                   # Entry point: Define your mission, bodies, and run simulation.
├── requirements.txt          # List of necessary Python libraries (NumPy, SciPy, Matplotlib).
├── .gitignore                # Rules for excluding temporary files and environments from Git.
└── README.md                 # Project documentation and usage guide.
