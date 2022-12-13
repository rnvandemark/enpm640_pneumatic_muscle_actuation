# enpm640_pneumatic_muscle_actuation

This project lives at:
https://github.com/rnvandemark/enpm640_pneumatic_muscle_actuation

Contains a number of ROS2 packages for simulation and control of a robot
consisting of pneumatic muscle actuators (PMA).

See the docs directory for a copy of the journal article referenced throughout
this project.

## Concise summary of each package's contents

### pma_description

Contains the output from the MoveIt Setup Assistant that was generated from a
simple 2-DoF planar robot arm's URDF. Comes complete with ros2_control hooks
and launch scripts that easily bring up a MoveIt interface with the PMA robot
model and, with a little tweaking, the custom ros2_control elements from this
project.

### pma_control

Only contains the custom trajectory controller node, which transforms an active
joint-space trajectory goal to 'pressure-space' by calculating the pneumatic
muscle pressures required to meet a joint position and velocity via
sliding-mode control.

### pma_hardware

Contains:
* The simulated hardware interface, which listens to the commanded values from
  the trajectory controller, and simulates the joint positions, velocities, and
  accelerations as a result.
* Data structures containing the overall simulation configuration (constant
  configuration parameters for the controller, each segment in the robot, and
  each pneumatic muscle in each segment), all of which are dynamically
  configurable on runtime.

### pma_interfaces

Contains definitions for simple ROS messages, at the moment consisting of
telemetry output by the custom trajectory controller.

### pma_util

Miscellaneous helper functions, basic type definitions, etc.

## Running the system

1. Install ROS2 Humble desktop (i.e. including RViz, MoveIt, ros2_control,
   ros2_action, etc.)
2. Clone this repository to an ament or otherwise compatible ROS2 workspace
3. Build the workspace and source it
4. Run 'ros2 launch pma_description demo.launch.py', which uses the custom
   trajectory controller and hardware interface
