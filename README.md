# AirDefense2025

Welcome to the AirDefense2025 project repository! This project focuses on developing systems for an air defense competition, including UI, code, schematics, and designs.

## Documentation

**All detailed documentation has been moved to the `/docs` directory.**

Please refer to the documentation for:

* **[Installation Instructions](./docs/installation/README.md)**
* **[Developer Guides (Git, ROS2)](./docs/guides/README.md)**
* **[Package Details](./docs/packages/README.md)**

## Quick Start (After Setup)

1.  Navigate to the workspace root: `cd path/to/AirDefense2025`
2.  Build the packages: `colcon build --symlink-install` (*Using symlink-install can speed up development for Python nodes*)
3.  Source the workspace: `source install/setup.bash`
4.  **To run the simulation:**
    ```bash
    # Example: Replace 'simulation.launch.py' with your actual sim launch file
    ros2 launch air_defense_sim simulation.launch.py
    ```
5.  **To run on hardware:**
    ```bash
    # Example: Replace 'main_bringup.launch.py' with your actual hardware launch file
    ros2 launch main main_bringup.launch.py
    ```

*(Consult the specific package documentation in `/docs/packages/` for details on available launch files and nodes.)*

## Repository Contents

* `/docs`: Detailed documentation.
* `/src`: ROS2 package source code (e.g., `main/`, `air_defense_sim/`).
* `ros2_install.sh`: Installation script for ROS2 Humble and tools.
* `ui_env_setup.sh`: Setup script for UI/Python dependencies (may need updates/review based on `main` package needs).
* Other configuration files, URDF, world files, etc.