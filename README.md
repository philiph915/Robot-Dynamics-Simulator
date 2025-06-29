# Robot Dynamics Simulator

A C++/Eigen-based simulator for articulated robots, implementing rigid body dynamics using Featherstone’s algorithms. This project is built for performance, modularity, and clarity — suitable for both learning and prototyping.

![robot_dynamics_simulator_test_window](https://github.com/user-attachments/assets/48e38236-f7ae-42fd-b94f-dd726ace0356)

## Features

- **Forward Kinematics (FK)** using Denavit-Hartenberg or custom link definitions
    
- **Recursive Newton-Euler Algorithm (RNEA)** for computing joint torques
    
- **Composite Rigid Body Algorithm (CRBA)** for computing the joint-space inertia matrix
    
- **Numerical Jacobian calculation** for end-effector velocity mapping
    
- **Raylib-based 3D visualization** for real-time feedback and debug overlays
    
- Modular structure for extension with controllers, sensors, or contact models
    

---

## Background

In robotics, dynamic modeling allows for simulation, planning, and control of articulated systems. This simulator implements two cornerstone algorithms from Featherstone’s _Rigid Body Dynamics Algorithms_:

### Recursive Newton-Euler Algorithm (RNEA)

RNEA solves the **inverse dynamics** problem: given joint positions `q`, velocities `𝑞̇`, and accelerations `𝑞̈`, it computes the required joint torques `τ`. It works in linear time by recursively traversing the robot’s kinematic chain.

- **Forward pass:** propagates link velocities and accelerations
    
- **Backward pass:** accumulates wrenches and computes torques
    

### Composite Rigid Body Algorithm (CRBA)

CRBA efficiently constructs the **mass matrix** `H(q)` by recursively aggregating link inertias and projecting them onto the motion subspaces. This is used for forward dynamics, control, and simulation of complex motion.

---

## Installation

### Prerequisites

- C++17 or later
    
- [Eigen](https://eigen.tuxfamily.org/) (header-only)
    
- [Raylib](https://www.raylib.com/)
    

### Build Instructions

```bash
git clone https://github.com/yourusername/robot-dynamics-simulator.git
cd robot-dynamics-simulator
mkdir build && cd build
cmake ..
make
```

---

## Running the Simulator

```bash
./build/Robot\ Dynamics\ Simulator
```

Or:

```bash
./build/Robot\ Dynamics\ Simulator
```

(Use backslashes to escape spaces, or quotes on macOS/Linux.)

Robot configurations can be customized by updating `InitLinks()` inside of `Robot.cpp`


---

## Example Scenario

Simulate a 3-link planar robot falling under gravity. Visual output includes:

- Joint positions and axes
    
- Link frames
    
- Real-time joint states and torques
    
- TBI: Optional debug overlays (e.g., Jacobians, center of mass)
    

---

## Future Work

- Contact and collision modeling
    
- Inverse kinematics and trajectory optimization
    
- GUI for robot configuration
    
- Logging and playback features
    

---

## References

- Roy Featherstone, _Rigid Body Dynamics Algorithms_
    
- Spong, Hutchinson, Vidyasagar, _Robot Modeling and Control_
    
- Siciliano et al., _Robotics: Modelling, Planning and Control_
    

---

## License

MIT License — use freely for education, research, and tinkering.

---
