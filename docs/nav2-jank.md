# On nav2 Jank

While working with nav2 (at least on Galactic) I discovered that the waypoint follower plugin has a bug where after 10 seconds of following a point, a tf transform exception causes it to die. 

After looking into the issue, I found that WF looks up the transform from the waypoint to the map in a cycle. The issue is that the stamp on the waypoint doesn't change, which means that the transform lookup will throw once the transform at that timestamp leaves the tf buffer (at 10 seconds by system default).

To solve this issue, I made a [fork of nav2](https://github.com/andyblarblar/navigation2/commit/2d5fc25dd64ec6db9ea95f34c4028d79ec00cfe6) that simply republishes each point every 3 seconds to keep the stamp current. Interestingly, simply updating the stamp on the point doesn't work, you need to republish it entirely. This works as waypoint follower has a preemption mechanism, which I am abusing to keep it going to the next point. While jank, this has allowed us to use nav2 for very long running scenarios fine, with the caveat that you can no longer interrupt with rviz points.