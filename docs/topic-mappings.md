# KiloOhm topic mappings
This is to be a document that outlines the namespacing of our topics. 
I would like this to be the one place you look to find where to get the LiDAR IMU, for example.

# Current configuration:
TODO very wip

### IMU topics
- `/kohm/gps/imu`: vectornav imu data
TODO lidar and realsense

### GPS topics
- `/kohm/navsat`: vectornav gps location

### Image topics
- `/kohm/camera_info`: Camera calibration from realsense
- `/kohm/image_raw`: images from realsense

### Point topics
- `/kohm/camera_points`: Points from WLD
- `/kohm/filtered_points`: filtered points from lidar proc
- `/kohm/combined_points`: the combined points of camera and filtered points
- `/scan`: the scan output from pc-ls
