# EVS for UAVs: Emulating Bio-Behavior via Anticipatory Active Inference and Data Augmentation

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20Melodic-blue)

This repository contains the official C++ implementation of the **Anticipatory Active Inference (AAI)** framework for UAV visual servoing.

This codebase accompanies our research paper: **"EVS for UAVs: Emulating Bio-Behavior via Anticipatory Active Inference and Data Augmentation"**. The proposed framework introduces Future Variational Free Energy (F-VFE) to enable proactive bio-inspired behavior, supported by a robust Data Augmentation strategy.

## 🌟 Key Contributions & Features

Based on the contributions of our paper, this repository provides:

1.  **Future-VFE (F-VFE) Metric**:
    * Extends Variational Free Energy (VFE) to the future time domain, enabling **anticipatory and proactive** system behaviors under uncertainty.
    * Unlike continuous active inference restricted to gradient descent, F-VFE embeds forward dynamics to allow for **analytic closed-form solutions**.
    * Casts active inference as a **constrained receding-horizon optimization**, naturally enforcing state/input constraints while improving global optimality.

2.  **Alternating Prediction Observe (APO)**:
    * Exploits the hybrid dynamics of UAV visual servo systems to act as a **"virtual soft sensor."**
    * Includes a novel **thrust estimation scheme** based on rotor speed telemetry to refine control input estimates and improve state estimation accuracy.
    * Delivers complete, **high-rate state updates** that match the high-frequency demands of active inference, significantly reducing prediction error compared to raw, low-rate visual feedback.

3.  **Comprehensive Open-Source Implementation**:
    * **Multi-Solver Support**: Covers both nonlinear and linear constrained optimization with efficient solvers tailored for different scales:
        * **Small/Medium-scale Dense**: Handled by **qpOASES** (Active-set).
        * **Large-scale Sparse**: Handled by **OSQP** (Operator splitting).
        * **Nonlinear**: Handled by **CasADi** (Symbolic framework).
    * **Data Augmentation**: Includes the proposed algorithm to enhance training and tracking robustness.
    * **Simulation & Soft-Hardware Platform**: Provides a solid foundation for the community to conduct rapid, reproducible experimentation.

## 📂 Code Structure

The core logic is divided into the main Data Association/Control node and modular Optimizer implementations:

* **`AAI_DA.cpp/h`**: The main ROS node logic. It handles:
    * **Sensor Interface**: Subscribes to VRPN/Mocap, IMU, rotor speed (`/mavros/esc_status`), and visual measurement topics.
    * **State Estimation**: Implements the `APO` class for filtering and state prediction using the thrust estimation scheme.
    * **Control Loop**: Manages the `xyzAxisControlLoop` thread and publishes acceleration commands (`/acc_cmd`).
* **Optimizer Implementations**:
    * `aai_qpOASES.cpp/h`: Implementation using **qpOASES** (Dense QP).
    * `aai_qp.cpp/h`: Implementation using **OsqpEigen** (Sparse QP).
    * `aai_casadi.cpp/h`: Implementation using **CasADi** (Nonlinear).
* **`run_AAI_DA_UAV.cpp`**: Entry point (main function) for the ROS node.

## 🛠️ Dependencies

Ensure you have the following dependencies installed in your environment:

* **ROS** (Noetic or Melodic recommended)
* **Eigen3** (Linear algebra)
* **qpOASES** (v3.2 or later)
* **OsqpEigen** & **OSQP**
* **CasADi** (C++ Interface)

### Installing Solvers (Examples)

```bash
# Install OsqpEigen
sudo apt install libosqp-dev
# (Follow OsqpEigen github instructions for C++ wrapper)

# Install CasADi
# (Download precompiled binaries or build from source via CMake)
```

## 🚀 Build & Usage

1.  **Clone the repository** into your Catkin workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/your-username/aai-uav-control.git
    ```

2.  **Build the package**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3.  **Run the Controller**:
    Modify the launch file (if provided) or run the node directly. Ensure your mavros or simulation environment is running.
    ```bash
    rosrun your_package_name run_AAI_DA_UAV
    ```

## ⚙️ Configuration

Key control parameters can be tuned in `AAI_DA.cpp` (or exposed via ROS params):

* **`Np` (Prediction Horizon)**: Length of the anticipatory window (e.g., 30 steps).
* **`dt`**: Sampling time (e.g., 0.025s).
* **`precice_z` / `precice_w`**: Precision weights balancing sensory evidence and internal model preferences.
* **`umin` / `umax`**: Control input (acceleration) constraints.

## 📊 Solver Selection

To switch between solvers, you currently need to link the corresponding `.cpp` file in your `CMakeLists.txt`. For example, to use **qpOASES**:

```cmake
add_executable(run_AAI_DA_UAV src/run_AAI_DA_UAV.cpp src/AAI_DA.cpp src/aai_qpOASES.cpp)
target_link_libraries(run_AAI_DA_UAV ${catkin_LIBRARIES} ${qpOASES_LIBRARIES})
```

*Change `src/aai_qpOASES.cpp` to `src/aai_qp.cpp` or `src/aai_casadi.cpp` depending on the desired backend.*
