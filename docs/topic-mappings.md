# KiloOhm topic mappings
This is to be a document that outlines the namespacing of our topics. 
I would like this to be the one place you look to find where to get the LiDAR IMU, for example.

# Current configuration:
TODO very wip

### IMU topics
- `/kohm/gps/imu`: vectornav imu data

### Odom topics 
- `/kohm/odom`: Currently the t265's VIO

### Encoder topics
- `/kohm/left_encoder_counts`: Left encoder values
- `/kohm/right_encoder_counts`: Right encoder values

### GPS topics
- `/kohm/navsat`: vectornav gps location
- `/kohm/mag`: vectornav magnetometer readings

### Image topics
- `/kohm/camera_info`: Camera calibration from realsense
- `/kohm/image_raw`: images from realsense depth

### Point topics
- `/sick/scan`: Raw scans from the SICK

The scan pipeline:
- `/kohm/camera_points`: Points from WLD
- `/kohm/filtered_points`: Points from the SICK driver
- `/kohm/combined_points`: the combined points of camera and filtered points
- `/scan`: the scan output from pc-ls

### Control topics
- `/robot/drive_mode`: Whether we are in teleop or auton
- `/cmd_vel`: Control vel from teleop. Passthrough is only allowed when in teleop by drive_mode_switch
- `/nav_vel`: Control vel from auton. Passthrough is only allowed when in auton by drive_mode_switch
- `/joy`: Teleop inputs
- 
