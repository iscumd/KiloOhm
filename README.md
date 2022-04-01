# KiloOhm

The next generation of Ohm, our IGVC bot.

## Installing Dependencies

You're going to want to have ROS2 installed. Here's the instructions for [Debian/Ubuntu 20](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

KiloOhm currently uses ROS2 Galactic.

If you are going to be running simulation, follow the Binary Install instructions for [Ignition Gazebo](https://ignitionrobotics.org/docs/fortress/install_ubuntu).

Install gflags by running:  `sudo apt-get install libgflags-dev`
	
## Building

> **NOTE:** Kohm uses a custom nav2 fork that requires all of nav2 to be built from source. This means that first builds can take upwards of 20 minutes
> on slower laptops. 

1. Change directory into your colcon workspace folder. Example: `cd ~/ros_ws/`
2. Clone the repository into your colcon workspace: `vcs import src --input https://raw.githubusercontent.com/iscumd/KiloOhm/master/kohm.repos`
3. Run rosdep to get ros dependencies:  
   `sudo rosdep init`  
   `rosdep update`  
   `rosdep install --from-paths src --ignore-src -y`  
4. Build the colcon workspace: `colcon build`
5. Source the local workspace: `. install/setup.bash`

## Launching

### Gazebo

`ros2 launch kohm_gazebo kohm.launch.py`

### IRL Robot

All features (autonomous and teleop):

`ros2 launch kohm_robot kohm.launch.py`

Minimal launch just requiring teleop and the roboteq:

`ros2 launch kohm_robot joystick_teleop.launch.py`

## Development

1. Create a issue describing a bug found or a feature to add.
2. Create a new branch or fork of the project. For branches, the name should describe the feature or bug you are trying to fix. Include a number that tags the issue that the change is associated with. Example: `git checkout -b feat/11-estop-state` or `git checkout -b bug/6-compile-errors` (11 & 6 being the issue number respectively)
3. Make changes on the new branch/fork.
4. Push changes to the branch/fork.
5. Confirm that it builds and solves the issue/implements the feature.
6. Make a pull request to merge the changes to the master branch, and link the appropriate issue to the pull request.
7. Have the pull request reviewed. Make any changes necessary to fix issues found.
8. Merge.

## Documentation

The docs directory will be filled with documention as we get closer to a stable kohm.

For each package used in kohm, the readme in the respective github repository should contain accurate documentation.
