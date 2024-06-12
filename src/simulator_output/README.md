# Simulator_output

## Run
1. copy all the required packages inside the `src` folder of a *ros2_workspace*
2. run `colcon build`
3. `ros2_workspace\install\setup.bat` 
4. execute `ros2 launch  simulator_output assetto_corsa_handler.py` ( to start also the receiver node)

or

4. execute `ros2 run  simulator_output simulator_output_IAC` ( to start only the simulator_output node)

## Notes
When running a simulation on Assetto Corsa without other cars, before launching this node you need to set the parameter `no_opponents` to **True** ( inside `launch\assetto_corsa_handler.py` or `simulator_output\simulator_output_IAC.py` when using the node alone ). 

This way the socket that listens for the data of the other opponents is disabled. 