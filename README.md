# AirDefense2025

Welcome to the AirDefense2025 project! This repository contains the UI, documentation, code, schematics, and designs for the air defense systems competition.

## Table of Contents
- [Installation Instructions](#installation-instructions)
  - [Installing ROS2](#installing-ros2)
  - [Setting Up the UI Environment](#setting-up-the-ui-environment)
- [General Workspace Information](#general-workspace-information)
  - [Building ROS Packages](#building-ros-packages)
  - [Running ROS Packages](#running-ros-packages)
- [Git and GitHub Basics](#git-and-github-basics)
- [Workspace Packages](#Workspace-Packages)
  - [cvpy](#cvpy)
  - [ui](#ui)

## Installation Instructions

### Installing ROS2

To install ROS2 using the provided script in this repository, follow these steps:

1. **Copy the code from the file** (if you haven't done so already):
   
   go to [ros2_install.sh](https://github.com/infinityplusone-teknofest/airdefense2025/blob/main/ros2_install.sh) and copy the code and then add to a new file
   ```bash
    nano ros2_install.sh 
    # paste the code 
   ```
   then press ctrl+x then y then enter to exit

2. **Make the installation script executable**:
   ```bash
   chmod +x ros2_install.sh
   ```

3. **Run the installation script with superuser privileges**:
   ```bash
   sudo -l  # This will prompt you for your password; enter it.
   ./ros2_install.sh
   ```

After completing these steps, you should have ROS2 (Jazzy) installed and running on your system, along with Visual Studio Code and other useful tools.

### Setting Up the UI Environment

To set up the UI environment necessary for running the user interface, follow these steps:

1. **Navigate to the repository directory** (if not already there):
   ```bash
   cd <repository-directory>
   ```

2. **Make the UI environment setup script executable**:
   ```bash
   chmod +x ui_env_setup.sh
   ```

3. **Run the UI environment setup script**:
   ```bash
   ./ui_env_setup.sh
   ```

## General Workspace Information

### Building ROS Packages

To build the ROS packages in this workspace, use the following command:

```bash
# Replace <your_workspace> with the actual workspace directory
cd <your_workspace>
colcon build
```

After building, source the setup file to overlay the workspace on your environment:

```bash
source install/setup.bash
```

### Running ROS Packages

To run the ROS packages, you can use the following command:

```bash
# Replace <your_package> with the actual package name
ros2 run <your_package> <your_node>
```

## Git and GitHub Basics

### Basic Git Commands

- **Initialize a local repository**:
  To create a new Git repository in your current directory, use:
  ```bash
  git init
  ```

- **Clone a repository**:
  To create a local copy of a remote repository, use:
  ```bash
  git clone <repository-url>
  ```

- **Check the status of your repository**:
  To see the current state of your working directory and staging area, use:
  ```bash
  git status
  ```

- **Add changes to the staging area**:
  To stage changes for the next commit, use:
  ```bash
  git add <file>  # Replace <file> with the name of the file you want to add
  ```
  To stage all changes, you can use:
  ```bash
  git add .
  ```

- **Commit your changes**:
  To save your staged changes to the repository, use:
  ```bash
  git commit -m "Your commit message here"
  ```

- **Push changes to the remote repository**:
  To upload your local commits to a remote repository, use:
  ```bash
  git push origin <branch-name>  # Replace <branch-name> with the name of your branch
  ```

- **Pull changes from the remote repository**:
  To fetch and merge changes from the remote repository into your current branch, use:
  ```bash
  git pull origin <branch-name>  # Replace <branch-name> with the name of your branch
  ```

- **Difference between clone, pull, and push**:
  - **Clone**: Creates a local copy of a remote repository. Use this when you want to start working on a project that is hosted on a remote server (e.g., GitHub).
  - **Pull**: Fetches changes from the remote repository and merges them into your current branch. Use this to update your local repository with changes made by others.
  - **Push**: Uploads your local commits to the remote repository. Use this to share your changes with others.

- **Change branches**:
  To switch to a different branch in your repository, use:
  ```bash
  git checkout <branch-name>  # Replace <branch-name> with the name of the branch you want to switch to
  ```
  To create a new branch and switch to it, use:
  ```bash
  git checkout -b <new-branch-name>  # Replace <new-branch-name> with the name of the new branch
  ```

- **Set up SSH key to connect Git with GitHub**:
  1. Generate a new SSH key (if you don't have one):
     ```bash
     ssh-keygen -t rsa -b 4096 -C "your_email@example.com"  # Replace with your email
     ```
     Press Enter to accept the default file location and enter a passphrase if desired.
  
  2. Add your SSH key to the SSH agent:
     ```bash
     eval "$(ssh-agent -s)"
     ssh-add ~/.ssh/id_rsa  # Replace with your key file if different
     ```

  3. Copy the SSH key to your clipboard:
     ```bash
     cat ~/.ssh/id_rsa.pub  # Replace with your key file if different
     ```
     Then copy the output.

  4. Add the SSH key to your GitHub account:
     - Go to GitHub, navigate to **Settings** > **SSH and GPG keys** > **New SSH key**.
     - Paste your key and give it a title, then click **Add SSH key**.

- **Add a remote repository**:
  To link your local repository to a remote repository, use:
  ```bash
  git remote add origin <repository-url>  # Replace <repository-url> with the URL of the remote repository
  ```

### Summary of Commands
- **Initialize**: `git init`
- **Clone**: `git clone <repository-url>`
- **Status**: `git status`
- **Stage**: `git add <file>` or `git add .`
- **Commit**: `git commit -m "message"`
- **Push**: `git push origin <branch-name>`
- **Pull**: `git pull origin <branch-name>`
- **Change Branch**: `git checkout <branch-name>` or `git checkout -b <new-branch-name>`
- **Add Remote**: `git remote add origin <repository-url>`
- **Set Up SSH**: Follow the steps outlined above.



## Workspace Packages

### cvpy
**This is a simple package designed to test the ros2 integration with opencv (make sure to have opencv installed or setup the ui enviroment and it will be installed automatically)**

*This package has two executables*

#### 1. image_publisher
Read Video Feed from the source specified in the source code (change to your need) and publishes the read frame to a topic called `image_data`

this executable takes an **optional** argument for the timer_period (a.k.a. delay between each video feed read)

*some good values for common fps(0.041 for 24fps , 0.033 for 30fps)*

```bash
ros2 run cvpy image_publisher [timer_period]
```
#### 2. image_subscriber

Receives an `image` message from a topic called `image_data` and displays it using opencv's `imshow`

```bash
ros2 run cvpy image_subscriber
```

### UI
**This is the UI package that we will be using for our project**

**Make sure to setup the ui enviroment first [Setting Up the UI Environment](#setting-up-the-ui-environment)**

*This package has two executables*

#### 1. publisher
Read Video Feed from the source specified in the source code (change to your need) and publishes the read frame to a topic called `image_data`

this executable takes an **optional** argument for the timer_period (a.k.a. delay between each video feed read)

*some good values for common fps(0.041 for 24fps , 0.033 for 30fps)*

```bash
ros2 run ui publisher [timer_period]
```
#### 2. ui

Receives an `image` message from a topic called `image_data` and displays alongside with other ui elements using `PyQt5`

```bash
ros2 run ui ui
```
