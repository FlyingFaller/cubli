

# Cubli Simulation

This is a Python-based demo of the Cubli, a 3D inverted pendulum that balances on a single corner using three orthogonally mounted reaction wheels.

The mathematical modeling, quaternion kinematics, and nonlinear attitude control architecture implemented in this code are directly based on the research paper *"The Cubli: Modeling and Nonlinear Attitude Control Utilizing Quaternions"* by Fabio Bobrow, Bruno A. Angelico, Flavius P. R. Martins, and Paulo S. P. da Silva.

## Core Code Architecture

The simulation is contained within the `smoking_gun.ipynb` notebook and is driven by a few key classes and functions:

### 1. Physics & Control (`CubliPhysics` class)

This class encapsulates the plant model and the control algorithms. Important methods include:

* `__init__(self, params)`: Initializes the physical parameters (mass, inertia tensors, gravity) and calculates the state regulator gains ($k_p$, $k_d$, $k_{p_w}$, $k_{d_w}$) required for stable feedback linearization.
* `friction_torque(self, omega_w)`: Calculates the environmental and motor viscous friction, heavily relying on a smooth approximation (like `np.tanh`) to prevent ODE solver stiffness.
* `check_wheel_saturation(self, tau_cmd, omega_w)`: Ensures the reaction wheels never exceed their velocity limits by tapering off commanded torque.
* `controller_fun(self, t, x)`: Implements the exact feedback linearization and quaternion-based state regulator to compute the required torque ($\vec{\tau}$) for the wheels to balance the structure.
* `ode_fun(self, t, x, tau_cmd)`: The core nonlinear equations of motion (EOMs) evaluated at each time step, computing $\dot{q}$, $\dot{\vec{\omega}}_c$, and $\dot{\vec{\omega}}_w$.

### 2. Simulation Engine

* `simulate_cubli(...)`: Acts as the main integration loop. It wraps `scipy.integrate.solve_ivp` to integrate `ode_fun` over time, while allowing for the injection of external torque disturbances.

### 3. Visualization

* `plot_cubli_simulation(t, x, tau)`: Generates 2D Matplotlib plots of the Cubli's states over time (Euler angles, body velocities, wheel velocities, and control torques).
* `animate_cubli_meshcat(...)`: Connects to a Meshcat server to render a real-time, interactive 3D visualization of the Cubli.

## Installation and Execution

This project uses `uv`. It is recommended that you clone the repository and use `uv sync` to install the correct dependencies. Then all the code and physics is in the `smoking_gun.ipynb` notebook.

## Example Outputs



## References

* F. Bobrow, B. A. Angelico, F. P. R. Martins, and P. S. P. da Silva, *"The Cubli: Modeling and Nonlinear Attitude Control Utilizing Quaternions,"* IEEE Access, vol. 9, 2021.

---