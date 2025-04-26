The original project by https://github.com/ProjectsWithRed/plotclock?tab=readme-ov-file is being modified.

To do (17.04.2025):
- Try the micropython code on the real setup (tick)
- If you can't make it work after calibration, you can use ESP32 and the original code. (tick, calibration is done)
- Maybe add a 3rd dof for rotation at the end effector with another servo so that you can adjust the angle of the impact surface. (tick, seems like not needed)
- Assemble the whole mechanism and try mode 1. (current task)
- Modify the design slightly. Remove the whiteboard, add legs, etc.
- Serial communication with the master and moving to the location (and angle of the impact surface) received by the master computer.
- Integrate the search according to the ball and target location.
