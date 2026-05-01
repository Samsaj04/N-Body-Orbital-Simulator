# N-Body Orbital Simulator

## Project Overview

**N-Body Orbital Simulator** is a Python-based tool for simulating and visualizing the motion of multiple celestial bodies under mutual gravitational attraction. It supports impulsive maneuvers (propulsion events) and provides animated visualizations of orbital trajectories in 2D or 3D.

## Key Features

- Simulate gravitational interactions between any number of bodies
- Add impulsive maneuvers (velocity changes) at specified times
- Visualize orbits and maneuvers with Matplotlib animations
- Supports both 2D and 3D simulations
- Customizable animation options (trail, speed, centering, relative mass display)

## Getting Started

### Installation

1. Clone this repository:
   ```bash
   git clone <your-repo-url>
   cd N-Body-Orbital-Simulator
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Usage

All simulation setup and execution is done in `main.py`. To run the simulator, simply execute:

```bash
python main.py
```

#### How to Use

1. **Create Body objects**
   - Each `Body` represents a celestial object (planet, satellite, etc.).
   - Required attributes: `position` (vector), `velocity` (vector), and `mass` (scalar).
   - Example:
     ```python
     earth = Body(position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]), mass=5.972e24)
     satellite = Body(position=np.array([r1, 0.0]), velocity=np.array([0.0, V_r1]), mass=1)
     ```

2. **Define Propulsion events (optional)**
   - Use `Propulsion` to specify maneuvers (impulses) for the spacecraft.
   - Attributes: `tf` (time of maneuver), `dVx`, `dVy`, `dVz` (velocity changes in each component).
   - Example:
     ```python
     impulse = Propulsion(tf=1000, dVx=0.5, dVy=0.0, dVz=0.0)
     ```

3. **SimulationController**
   - This class manages the simulation, integrating all bodies and maneuvers.
   - Required attributes:
     - `bodies`: List of all `Body` objects (the last one should be the spacecraft if it performs maneuvers)
     - `G`: Gravitational constant
     - `ti`: Initial time
     - `tf`: Final time
     - `step`: Time step (affects simulation resolution)
     - `impulse`: List of all `Propulsion` events (optional)
   - Example:
     ```python
     controller = SimulationController(
         bodies=[earth, satellite],
         G=G,
         ti=0,
         tf=10000,
         step=20,
         impulse=[impulse]
     )
     ```
   - **Note:** If you have a spacecraft performing maneuvers, it must be the last `Body` in the list. Currently, impulses are only applied to the last body.

4. **Run the simulation**
   - Call `run_solution()` on your `SimulationController` object to compute the trajectories:
     ```python
     orbits = controller.run_solution()
     ```

5. **Visualization**
   - Create a `Visualizer` object to plot or animate the mission:
     - `bodies`: List of bodies
     - `trajectories`: Output from `run_solution()`
     - `dim`: 2 or 3 (for 2D or 3D)
     - `follow`: Controls the trail length (higher = longer trail)
     - `speed`: Animation speed
     - `centered`: If `True`, the plot zooms to follow the bodies; if `False`, shows the full domain
     - `rel_mass`: If `True`, marker size is proportional to mass
   - Example:
     ```python
     viz = Visualizer(
         bodies=[earth, satellite],
         trajectories=orbits,
         dim=2,
         follow=np.inf,
         speed=5,
         centered=False,
         rel_mass=False
     )
     ```
   - To plot a static image:
     ```python
     viz.plotting(True)
     ```
   - To animate:
     ```python
     viz.animate()
     ```

## Support & Documentation

- For questions or issues, please open an issue in this repository.

## Maintainers & Contributions

- Maintained by: Samuel Jiménez Arroyave
- Contributions are welcome! Please submit pull requests or open issues for suggestions.

---

*This project is open source and welcomes community contributions.*