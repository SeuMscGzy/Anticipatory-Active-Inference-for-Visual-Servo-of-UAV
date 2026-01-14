
# Hardware Architecture & Experimental Platform

To ensure reproducibility and facilitate further research in agile UAV control, we provide a detailed breakdown of our custom-built quadrotor platform and the thrust measurement apparatus used for system identification.

## 1. Agile Quadrotor Platform

Our experimental platform is a custom-designed, high-performance quadrotor optimized for indoor visual servoing and agile maneuvers. The system features a high thrust-to-weight ratio (> 4.0) and onboard edge computing capabilities.

![UAV Hardware Architecture](uav.jpg)

### Component Specifications

| Component | Model / Description | Note |
| :--- | :--- | :--- |
| **Frame** | Custom T300 Carbon Fiber | Open-sourced design (see CAD files below) |
| **Onboard Computer** | Orange Pi 5 Pro | Runs ROS, Image Processing, and NMPC |
| **Vision Sensor** | Intel RealSense D405 | High-resolution close-range depth/RGB |
| **Flight Controller (FCU)**| Holybro Kakute H7 v1.5 | Low-level attitude control |
| **Motors** | T-Motor F2004 (KV3000) | High KV for agility |
| **ESC** | Tekko32 F4 4-in-1 ESC (50A) | DShot protocol support |
| **Propellers** | EMAX Silver Swallow 3528 | 3.5-inch props |
| **Battery** | Bosch LiPo 4S (1050 mAh) | High discharge rate |
| **Thrust-Weight Ratio** | **> 4.0** | Critical for aggressive maneuvers |

### System Architecture
The hardware architecture is designed for low-latency control:
* **High-Level Control**: The **Orange Pi 5 Pro** handles the Active Inference (AAI) controller and image processing. It communicates with the FCU via **Mavros/ROS**.
* **Low-Level Control**: The **Kakute H7** executes attitude commands.
* **Remote Management**: System monitoring and emergency control are handled via **NoMachine** (WiFi) and a standard RC transmitter.

---

## 2. Thrust Test Platform

To achieve precise model-based control (as required by the AAI framework), accurate identification of the propulsion system parameters (thrust coefficients $k_1, k_2, k_3$) is essential. We designed a custom thrust test stand to collect ground truth data mapping rotor speed (RPM) to thrust.

![Thrust Test Platform](thrust test platform.jpg)

### Features
* **Mechanism**: A lever-arm structure with a sliding rail ensures linear force transmission to the load cell.
* **Data Acquisition**: A dedicated DAQ system records real-time thrust values corresponding to different throttle/RPM inputs.
* **Calibration**: Used to generate the thrust curve model: $T = k_1 \omega^2 + k_2 \omega + k_3$.

---

## 3. Open Source Hardware Resources

We provide the 3D model files to assist the community in replicating our setup or building similar platforms.

### 📂 File Descriptions

#### UAV Frame (SolidWorks Parts)
The custom T300 carbon fiber frame consists of the following modular parts:
* **`baseborad.SLDPRT`**: The main bottom plate for mounting the battery and ESC.
* **`top plate.SLDPRT`**: The upper structural plate for flight controller protection and camera.
* **`tripod.SLDPRT`**: The landing gear/leg design.
* **`upboard for extra devices.SLDPRT`**: An extension plate for mounting the other sensors such as gps.

#### Thrust Test Stand
* **`thrust test platform.STEP`**: The complete assembly model of the thrust testing apparatus, ready for CNC machining or 3D printing guidance.

### How to Use
* **SolidWorks Files (.SLDPRT)**: Can be opened and modified in SolidWorks for CNC cutting (recommended material: T300 Carbon Fiber, thickness 1.5mm - 2.0mm).
* **STEP Files (.STEP)**: Universal 3D format compatible with Fusion 360, FreeCAD, and other CAD tools.
