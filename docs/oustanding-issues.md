### The many outstanding issues of Ohm

Ohm is a troubled robot. Despite our efforts, some bugs persist and must be lived with. This file is a documentation of these issues.

## No points show up on boot

Occasionally, no points (and thus map) will show up on boot. This occurs because the realsense failed to boot, thus not allowing
pointcloud concat to publish scan messages. This has no known fix, so just reboot.

## Tf blows up with nan quaternions

For some reason, after sitting still for long enough tf will explode the log with a nan quaternion error. This kills everything, and seems to happen after sitting still for a long enough period of time. This is a pretty big issue, but not for IGVC.

## The lidar doesn't connect

Make sure the laptop is off wifi and the ethernet cable is plugged in. Sometimes the ethernet network fails to attach. In this case, just restart the laptop.

## GPS points are sent to random places

Currently I cannot figure out how to get heading out of the GPS. Because of this, the bot must be turned on facing north for gps points to be in the correct positions. You can start auton anywhere however, as we can use a tf transform to correct for movement.