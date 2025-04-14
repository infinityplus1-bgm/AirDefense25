# Installing ROS2 (Humble)

This guide details how to install ROS2 Humble using the `ros2_install.sh` script provided in the main repository directory. This script also installs Visual Studio Code and other useful tools. **Note:** Ensure the `ros2_install.sh` script you are using is indeed configured to install ROS2 Humble.

## Steps

1.  **Get the script:**
    Navigate to the root directory of the `AirDefense2025` repository clone. If you don't have the script locally, you can copy its content from [ros2_install.sh on GitHub](https://github.com/infinityplusone-teknofest/airdefense2025/blob/main/ros2_install.sh). Create a local file:
    ```bash
    nano ros2_install.sh
    # Paste the script content here
    ```
    Save and exit (Ctrl+X, then Y, then Enter).

2.  **Make the script executable:**
    ```bash
    chmod +x ros2_install.sh
    ```

3.  **Run the script with administrator privileges:**
    You'll need to enter your user password.
    ```bash
    sudo ./ros2_install.sh
    ```

Upon successful completion, ROS2 Humble should be installed and configured on your system. You can verify the installation by opening a new terminal and trying a command like `ros2 --help`. Remember to source the ROS2 environment in new terminals if it wasn't added automatically to your `.bashrc` (the script usually handles this): `source /opt/ros/humble/setup.bash`.