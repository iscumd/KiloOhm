### To Setup The Extended Kalman Filter
Use the sample EKF config file for an explanation and detailed description of each of the ekf parameters. Example EKF config can be found [here!](https://github.com/cra-ros-pkg/robot_localization/blob/galactic-devel/params/ekf.yaml) each sensor will require its own matrix of variables that you want to include. Since KiloOhm is a robot designed primarily to operate in a 2D planar environment make sure that no 3D variables are being fused into the EKF. Examples would be: Z position, velocity and acceleration, and roll and pitch as well as there respective velocities. This should ensure that KiloOhm will stay within the 2D map. In addition make sure that the two_d_mode in the param file is set to true so that the EKF wont publish any change in Z, roll or pitch.

### Using The Extended Kalman Filter
Using robot localization's Extended Kalman Filter is simply a matter of launching it using a launch file that points to the params file you created above. Please note that the EKF node will provide the odom -> base_footprint transform so please ensure that no other odometry source provides a odom -> base_link transform!

### Intel Realsense T265 Settings for Use With The EKF
To use the T265 tracking camera with the EKF you must ensure that both launch parameters `publish_tf` and `publish_odom_tf` are set to false. The `publish_tf` param controls whether or not to publish the static transforms between the builtin cameras and the odom frame. Since we want to use a custom frame for the entire camera we don't want this to be published. The `publish_odom_tf` param controls whether the T265 will provide a transform from the specified odom frame to the specified base_link frame. If this is left true then this will result in a tf conflict between the transform published by the ekf and the transform published by the Realsense.

### EKF Issues
We ran into a couple of issues while trying to use the ekf including KiloOhm being transformed above the costmap which was a result of fusing 3D variables into the EKF node. This issues manifested for us a the error `sensor origin is out of map frame`. This error will also manifest if whichever node providing the odom -> base_footprint transform is transforming Z, roll or pitch. In our case the Intel Realsense T265 tracking camera which was providing the odom -> base_footprint transform was transforming KiloOhm in 3D resulting in the sensor origin issue.

### Doc!
For more information regarding explanation, setup and configuration of the robot localization node please see the docs [here!](https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node)

