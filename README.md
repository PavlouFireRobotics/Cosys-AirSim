# Cosys-AirSim (PavlouFireRobotics Fork)

Cosys-AirSim Unreal plugin and ROS2 wrapper.

## Related Repositories
- Unreal Environment: https://github.com/PavlouFireRobotics/Unreal-Fire-Simulation
- Perception Stack: https://github.com/PavlouFireRobotics/perception-detection-python

## Build (Windows)

This section describes how to build **Cosys-AirSim** on Windows using **Unreal Engine 5.5.4** and **Visual Studio 2022**.

---

### Requirements

- Windows 10 or Windows 11
- Unreal Engine **5.5.4**
- Visual Studio **2022** (⚠️ do **not** use Visual Studio 2026)

---

### Step 1 — Install Unreal Engine 5.5.4

1. Download and install the **Epic Games Launcher**  
   https://www.unrealengine.com/download

2. Open the Epic Games Launcher and sign in (Epic account required).

3. In the launcher:
   - Go to **Unreal Engine** → **Library**
   - Click **Install Engine**
   - Select **Unreal Engine 5.5.4**
   - Choose an installation directory and complete the installation

4. Once installed, make sure **5.5.4** is set as the active version.

---

### Step 2 — Install Visual Studio 2022

1. Download **Visual Studio 2022** from the official Microsoft page:  
   https://learn.microsoft.com/en-us/visualstudio/releases/2022/release-history#release-dates-and-build-numbers

2. Choose **Community** (or **Build Tools** if you prefer a minimal install).

3. During installation, select the following:

- ✅ **Desktop development with C++**
- ✅ **Windows 10/11 SDK (10.0.x – latest available)**
- ✅ **MSVC v143 - VS 2022 C++ x64/x86 build tools**
- ✅ **.NET Framework SDK (latest available)**

4. Complete the installation and restart your machine if prompted.

---

### Step 3 — Build Cosys-AirSim

1. Open **Developer Command Prompt for VS 2022**  
   (Start Menu → Visual Studio 2022 → Developer Command Prompt)

2. Clone the repository:
   ```bat
   git clone https://github.com/PavlouFireRobotics/Cosys-AirSim.git
   cd Cosys-AirSim
   build.cmd
    ```


## ROS2 Wrapper (Ubuntu 22.04, ROS2 Iron)

```bash
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update

cd <path-to-cosys-airsim>/ros2
rosdep install --from-paths src -y --ignore-src --skip-keys pcl --skip-keys message_runtime --skip-keys message_generation

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch airsim_ros2_teleop teleop_airsim.launch.py
```
