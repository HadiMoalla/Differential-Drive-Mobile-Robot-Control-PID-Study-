# 🧭 User Manual – Robot Control & Simulation

## 1. Overview

This project provides a control framework for robot navigation using a PID-based approach. It includes:

- A **main simulation script** (MATLAB file)
- A **parameter file** for tuning and configuration
- Multiple **Simulink models** for testing different strategies

The system allows users to simulate robot motion, tune controller gains, and analyze performance under different configurations.

---

## 2. Project Structure

```
project/
│── README.md
│── src/
│   │── Simulation_code.m
│   │── parameters.m
│   │── Control_sim_constV.slx
│   │── Control_sim_FixedV.slx
│   │── Ziegler_Nichols_closed_Loop.slx
│   │── Ziegler_Nichols_open_Loop.slx
│── figures/
```

---

## 3. Requirements

Before running the project, make sure you have:

- MATLAB (recommended R2021 or later)
- Simulink (for simulation)
- Control System Toolbox (for PID tuning)

---

## 4. Setup Instructions

### Step 1: Clone or Download the Repository

```bash
git clone https://github.com/HadiMoalla/Differential-Drive-Mobile-Robot-Control-PID-Study-.git
```

---

### Step 2: Open MATLAB

Navigate to the project folder:

```matlab
cd('path_to_project')
addpath('src')
```

---

### Step 3: Load Parameters

Run the parameter file to initialize all variables:

```matlab
parameters
```

This step is required before running any simulation or controller code.

---

## 5. Running the Code

### Option 1: MATLAB Script Execution

Run the main simulation script:

```matlab
Simulation_code
```

This will:

- Initialize the robot state
- Apply control laws
- Generate trajectory and response plots

---

### Option 2: Simulink Models

You can choose between multiple Simulink configurations:

- Constant velocity model:
```matlab
open('src/Control_sim_constV.slx')
```

- Velocity model with adaptive Vmax:
```matlab
open('src/Control_sim_FixedV.slx')
```

- Ziegler–Nichols tuning (closed loop):
```matlab
open('src/Ziegler_Nichols_closed_Loop.slx')
```

- Ziegler–Nichols tuning (open loop):
```matlab
open('src/Ziegler_Nichols_open_Loop.slx')
```

Before running any model:

```matlab
parameters
```

Then click **Run** inside Simulink.

---

## 6. Parameter Tuning

All key parameters are defined in:

```
src/parameters.m
```

### Important Parameters

- `Kp`, `Ki`, `Kd` → PID gains
- `Vmax` → maximum linear velocity
- `threshold_distance` → switching point for velocity scaling
- `initial_conditions` → robot starting state

---

### Using PID Tuner

1. Open any Simulink model
2. Select the PID block
3. Click **Tune**
4. Adjust:
   - Response speed
   - Overshoot
   - Stability
5. Export tuned gains back to `parameters.m`

---

## 7. Output & Results

After running the simulation, you should see:

- Robot trajectory plots
- Heading error (α) vs time

Figures are saved or displayed depending on your script configuration.

---

## 8. Extending the Project

You can extend this work by:

- Adding obstacle avoidance
- Integrating sensor feedback
- Testing different control strategies (e.g., MPC, adaptive control)

---

## 9. Contact

For questions or improvements, feel free to open an issue or contribute to the repository.

---

