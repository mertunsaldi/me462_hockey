The original project by https://github.com/ProjectsWithRed/plotclock?tab=readme-ov-file is being modified.

To do (17.04.2025):
Done:
- Try the micropython code on the real setup (tick)
- If you can't make it work after calibration, you can use ESP32 and the original code. (tick, calibration is done)
- Maybe add a 3rd dof for rotation at the end effector with another servo so that you can adjust the angle of the impact surface. (tick, seems like not needed)
- Assemble the whole mechanism and try mode 1. (tick, small design changes and modifications to code are done)
To be done:
- 0,0 noktasinin yerini ve workspace'ini tam olarak belirle (Hangi koseye uzanabiliyor ve konumu ne vs.) Kodda da bu duzgun bir sekilde map edilmis olmali. 
- Try going to the desired location in a simulation environment. 
- Add a ball and target location to the simulation and integrate the search according to the ball and target location.
- Serial communication with the master and moving to the location (and angle of the impact surface) received by the master computer.
- Modify the design slightly. Remove the whiteboard, add legs, etc.



Su an 0,10 diyince y'de 0'a geliyor. 0,5 diyince servo baseini donduruyor o yuzden y=10 benim sifirim olmali. 0,10 ile  0,20 arasinda yaklasik 11mm mesafe var. Boyle bir errorum var gibi duruyor.
