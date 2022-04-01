## To test Ohm with teleop:
1. Connect the computer to the roboteq with its usb cable.
2. In the computer open a terminal with ctrl-alt-t.
3. Type `cd kohm_ws` (or whatever the current ros workspace is).
4. Then `colcon build` the first time you run.
5. When done, `. install/setup.bash`. Do this any time you build.
6. Finally, `ros2 launch kohm_robot joystick_teleop.launch.py`.

This should bring up rviz and start running the bot. From here on out, any controller inputs will be sent to the roboteq.

You can either control using the xbone controller wired into ohm, or use the wii remote.

## To use the wii remote (do this before running the bot):
1. Open a new terminal in the same directory as above.
2. `. install/setup.bash`
3. `ros2 launch wii_wheel wheel_teleop.launch.py`
4. Now it should prompt you to press 1 and 2 on a wii remote, so find ours and connect the nunchuck.
5. Press 1 and 2 with the remote on a flat surface, and wait till the terminal prints activated wiimote or something like that.
6. The remote should now work! see https://youtu.be/aM5cF3BLd4k for instructions on use.
