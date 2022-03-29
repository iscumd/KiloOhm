# KiloOhm topic mappings
This is to be a document that outlines the namespacing of our topics. 
I would like this to be the one place you look to find where to get the LiDAR IMU, for example.

# Current configuration:
TODO very wip

### IMU topics
- `/kohm/gps/imu`: vectornav imu data
- `/kohm/realsense/imu`: Currently the t265's IMU

### Odom topics 
- `/kohm/odom`: Currently the t265's VIO

### GPS topics
- `/kohm/navsat`: vectornav gps location

### Image topics
- `/kohm/camera_info`: Camera calibration from realsense
- `/kohm/image_raw`: images from realsense

### Point topics
- `/sick/scan`: Raw scans from the SICK
- `/kohm/camera_points`: Points from WLD
- `/kohm/filtered_points`: Points from the ls-pc conversion of /sick/scan
- `/kohm/combined_points`: the combined points of camera and filtered points
- `/scan`: the scan output from pc-ls
