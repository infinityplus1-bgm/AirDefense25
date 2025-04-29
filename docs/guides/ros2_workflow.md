# ROS2 Workspace Workflow

This guide covers the basic commands for working with the ROS2 packages within this (`AirDefense2025`) workspace.

## Building Packages

Before running any custom nodes or launch files, you need to build the packages in your workspace using `colcon`, the standard ROS2 build tool.

1.  **Navigate to the workspace root:**
    Open a terminal and change directory to the root of your ROS2 workspace (the directory containing the `src` folder, e.g., `~/AirDefense2025/`).
    ```bash
    cd path/to/your_workspace # e.g., cd ~/AirDefense2025
    ```

2.  **Build the workspace:**
    Run the `colcon build` command.
    ```bash
    colcon build --symlink-install
    ```
    *   This command discovers and builds all ROS2 packages found in the `src` directory.
    *   Build artifacts are placed in `build/`, installed files in `install/`, and logs in `log/`.
    *   `--symlink-install` allows you to edit Python files or non-compiled resources (like launch files, config files) directly in the `src` directory and have the changes reflected without rebuilding (though you still need to restart the nodes/launch files). For C++ changes, a rebuild is always necessary.

## Sourcing the Workspace

After building, you need to source the `setup` file generated in the `install` directory. This updates your current terminal session's environment variables so that ROS2 tools can find your custom packages, nodes, launch files, and messages.

```bash
source install/setup.bash
```
Or, if you use `zsh`:
```bash
source install/setup.zsh
```

**Important:** You must run this `source` command in *every new terminal* where you intend to use commands related to this workspace (like `ros2 run` or `ros2 launch`). Consider adding this line to your shell's startup file (e.g., `~/.bashrc` or `~/.zshrc`) for convenience, but be aware this can cause issues if you frequently switch between different ROS2 workspaces.

## Running Nodes

To run a single executable (a ROS2 node) from a package, use the `ros2 run` command after building and sourcing the workspace.

```bash
ros2 run <package_name> <executable_name> [arguments...]
```

*   `<package_name>`: The name of the package containing the node (e.g., `main` , `simulation` ).
*   `<executable_name>`: The name of the executable as defined in the package's `setup.py` or `CMakeLists.txt` (e.g., `image_publisher`,  `camera_node`).
*   `[arguments...]`: Optional command-line arguments passed to the node's main function.

**Example:**
To run the `image_publisher` node from the `cvpy` package:
```bash
ros2 run main camera
```

## Launching Systems

To start multiple nodes, load parameters, and configure the system as a whole, use ROS2 launch files (`.launch.py`) with the `ros2 launch` command.

```bash
ros2 launch <package_name> <launch_file_name> [launch_arguments...]
```

*   `<package_name>`: The name of the package containing the launch file.
*   `<launch_file_name>`: The name of the Python launch file (e.g., `main_bringup.launch.py`, `simulation.launch.py`).
*   `[launch_arguments...]`: Optional arguments passed to the launch file, often used to configure parameters or specify modes (e.g., `world:=my_world.world`).

**Example:**
To launch the main system bringup file (assuming it's named `main_bringup.launch.py` in the `main` package):
```bash
ros2 launch main main_bringup.launch.py
```


