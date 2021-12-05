# SAFE LANDING PLANNER
https://github.com/PX4/PX4-Avoidance

ROS node for safe drone landing site detection and waypoint generation.

The *safe_landing_planner* classifies the terrain underneath the vehicle based on the mean and standard deviation of the z coordinate of pointcloud points. The pointcloud from a downwards facing sensor is binned into a 2D grid based on the xy point coordinates. For each bin, the mean and standard deviation of z coordinate of the points are calculated and they are used to locate flat areas where it is safe to land.


# Getting Started

## Installation

1. Ubuntu 18.04 (Bionic Beaver) 

1. ROS Melodic

1. MAVROS

1. Clone and build this repository in your catkin workspace.
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AnujAgrawal7/avoidance.git
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```
   
1. PX4 Firmware (https://github.com/PX4/PX4-Autopilot)

## Run the Avoidance Gazebo Simulation

### Build and Run the Simulator

1. Build the Firmware once in order to generate SDF model files for Gazebo.
   This step will actually run a simulation (that you can immediately close).

   ```bash
   # This is necessary to prevent some Qt-related errors (feel free to try to omit it)
   export QT_X11_NO_MITSHM=1

   # Build and run simulation
   make px4_sitl_default gazebo
   
   # Quit the simulation (Ctrl+C)

   # Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
   . ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
   ```

1. Add the Firmware directory to ROS_PACKAGE_PATH so that ROS can start PX4:
   ```bash
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot
   ```
1. Finally, set the GAZEBO_MODEL_PATH in your bashrc:
   ```bash
   echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds" >> ~/.bashrc
   ```

The last three steps, together with sourcing your catkin **setup.bash** (`source ~/catkin_ws/devel/setup.bash`) should be repeated each time a new terminal window is open.



### Safe Landing Planner

This section shows how to start the *safe_landing_planner* and use it to land safely in mission or auto land mode.
To run the node:

 Set the parameter `COM_OBS_AVOID` to 1 in QGroundControl.

```bash
roslaunch safe_landing_planner safe_landing_planner.launch
```

You will see an unarmed vehicle on the ground. Open [QGroundControl](http://qgroundcontrol.com/), either plan a mission with the last item of type *Land* or fly around the world in Position Control, click the *Land* button on the left side where you wish to land.
At the land position, the vehicle will start to descend towards the ground until it is at `loiter_height` from the ground/obstacle. Then it will start loitering to evaluate the ground underneeth.
If the ground is flat, the vehicle will continue landing. Otherwise it will evaluate the close by terrain in a squared spiral pattern until it finds a good enough ground to land on.

# Run on Hardware

## Prerequisite

### Camera

The planner require a 3D point cloud of type `sensor_msgs::PointCloud2`. Any camera that can provide such data is compatible.

The officially supported camera is Intel Realsense D435. We recommend using Firmware version 5.9.13.0. The instructions on how to update the Firmware of the camera can be found [here](https://www.intel.com/content/www/us/en/support/articles/000028171/emerging-technologies/intel-realsense-technology.html)

> **Tip:** Be careful when attaching the camera with a USB3 cable. USB3 might might interfere with GPS and other signals. If possible, always use USB2 cables.

Other tested camera models are: Intel Realsense D415 and R200, Occipital Structure Core.


### PX4 Autopilot

Parameters to set through QGC:
* `COM_OBS_AVOID` to Enabled
* `MAV_1_CONFIG`, `MAV_1_MODE`, `SER_TEL2_BAUD` to enable MAVLink on a serial port. For more information: [PX4 Dev Guide](http://dev.px4.io/en/companion_computer/pixhawk_companion.html#pixhawk-setup)

### Companion Computer

* OS: Ubuntu 18.04 OS 
* ROS Melodic: see [Installation](#installation)
* Other Required Components for Intel Realsense:
  - Librealsense (Realsense SDK). The installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
  - [Librealsense ROS wrappers](https://github.com/intel-ros/realsense.git)




## Safe Landing Planner
```
roslaunch safe_landing_planner safe_landing_planner_launch_real_sense.launch
```

In the `cfg/` folder there are camera specific configurations for the algorithm nodes. These parameters can be loaded by specifying the file in the `VEHICLE_CONFIG_SLP` and `VEHICLE_CONFIG_WPG` system variable for the safe_landing_planner_node and for the waypoint_generator_node respectively.

The size of the squared shape patch of terrain below the vehicle that is evaluated by the algorithm can be changed to suit different vehicle sizes with the WaypointGeneratorNode parameter `smoothing_land_cell`. The algorithm behavior will also be affected by the height at which the decision to land or not is taken (`loiter_height` parameter in WaypointGeneratorNode) and by the size of neighborhood filter smoothing (`smoothing_size` in LandingSiteDetectionNode).

For different cameras you might also need to tune the thresholds on the number of points in each bin, standard deviation and mean.

