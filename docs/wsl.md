
# WSL2 (Ubuntu 24.04) & Dynamixel MX64 Setup Guide

## Prerequisites

1.  Windows 10 or Windows 11
2.  WSL2 installed and enabled
3.  Ubuntu 24.04 WSL2 
4.  ROS2 installed within Ubuntu 24.04 WSL2
5.  Essential build tools in WSL2: `sudo apt update && sudo apt install build-essential`
6.  Dynamixel MX64, USB interface 

## Part 1: Windows 

1.  **Install `usbipd-win` (PowerShell as admin):**
    ```powershell
    winget install --interactive --exact dorssel.usbipd-win
    ```

2.  **Identify USB device BUSID:**
    ```powershell
    usbipd list
    ```
3.  **Bind USB device:**
    *(Replace `<YOUR_BUSID>` with the actual bus ID)*
    ```powershell
    usbipd bind --busid <YOUR_BUSID>
    ```

4.  **Attach device to WSL2:**
    ```powershell
    usbipd attach --wsl --busid <YOUR_BUSID>
    ```
## Part 2: WSL2 configuration & build

5.  **Verify Device Access in WSL2:**
    ```bash
    ls /dev/ttyUSB*
    dmesg | tail
    ```
6.  **ROS2 setup, build, run**
7.  **git clone this repo into ws**
8.  **Init submodules (DynamixelSDK)**
9.  **Build DynamixelSDK for Linux64 C++ (make, sudo make install)**
10.  **Build ros2 node (Colcon build)**
11.  **Source ros2 node, run it.*