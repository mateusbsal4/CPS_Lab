# Cyber-Physical Systems Lab

This repository contains the code for the **Cyber-Physical Systems (CPS) practical course** taken at TUM. It contains two ROS workspaces, described below.

## Simulator

This workspace contains the implementation of the follow-the-gap controller to autonomously drive the F110 car in simulation. To run the simulation, follow these steps:

1. Navigate to the simulator folder:
    ```bash
    cd simulator
    ```

2. Build the ROS project using `catkin_make`:
    ```bash
    catkin_make
    ```

3. Launch the simulator:
    ```bash
    roslaunch f110_simulator simulator.launch
    ```

Once the simulation is running:

- Press **`f`** to select the autonomous controller.
- Press **`k`** to allow manual control of the car.
- The system includes an **automatic emergency braking** feature, preventing the car from crashing into the track walls.

## Project

This workspace contains the implementation of a **PID controller** to autonomously drive a 1:10 scale F110 miniature car equipped with a 2D LiDAR sensor around a race track.

To run the controller on the F110 car:

1. Connect to the vehicle via ssh
2. Navigate to the project folder:
    ```bash
    cd project
    ```
3. Build the ROS project using `catkin_make`:
    ```bash
    catkin_make
    ```
4. Run the general car launch:
    ```bash
    roslaunch racecar teleop.launch
    ```
5. Launch the controller package:
    ```bash
    roslaunch relampago relampago.launch
    ```
    
The controller was tested, and the results can be seen in the video below:

[View the test video](https://github.com/user-attachments/assets/1f92fa13-c9eb-4a28-8e78-7def6f82d74c)
