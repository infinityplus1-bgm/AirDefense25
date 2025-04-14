# Setting Up the UI Environment

The project's user interface requires specific dependencies. Use the provided `ui_env_setup.sh` script to install them.

## Steps

1.  **Navigate to the repository directory:**
    Ensure you are in the root directory of your cloned `AirDefense2025` repository.
    ```bash
    cd path/to/AirDefense2025
    ```

2.  **Make the setup script executable:**
    ```bash
    chmod +x ui_env_setup.sh
    ```

3.  **Run the setup script:**
    ```bash
    ./ui_env_setup.sh
    ```

This script will install Python packages (like PyQt5, OpenCV-Python if needed) and any other dependencies required by the UI package.