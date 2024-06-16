# assetto_corsa_ros2
ROS2 Wrapper to communicate with Assetto Corsa racing simulator.
These interfaces are part of the paper: [A Simulation Benchmark for Autonomous Racing with Large-Scale Human Data](https://dasgringuen.github.io/assetto_corsa_gym/).
Main repo of the paper: [assetto_corsa_gym](https://github.com/dasGringuen/assetto_corsa_gym)

Requirements:
- Assetto Corsa
- ROS2 (it has been tested on ROS2 Humble but should work with other versions)
- Vjoy (virtual controller)
- Plugin and Windows libraries needed

1. Install Assetto Corsa
    - Download and install [Steam](https://store.steampowered.com/)
    - Buy and install [Assetto Corsa](https://store.steampowered.com/app/244210/Assetto_Corsa) (not Assetto Corsa Competizione). I suggest to buy the Ultimate Edition pack.
<be>

2. Plugin and Windows libraries needed
    - Follow the steps here [link](https://github.com/dasGringuen/assetto_corsa_gym/blob/main/INSTALL.md)
<be>

3. Install ROS2
    - Follow the steps here [link]([https://docs.ros.org/en/foxy/Installation/Windows-Development-Setup.html](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html))
    - Be sure to source the ROS2 environment in each terminal you use.
    - I strongly recommend installing and using the Windows PowerShell to have multiple terminals in a unique window and to exploit other useful tricks.
<be>

4. Run Assetto Corsa
    - Check if you have the correct game settings: gMeter and sensors_par in options->general should be activated
    - Check the options are correct, in options → control, be sure to have Vjoy and WASD
    - Select the car, track, and race mode (practice starts from the pitlane, laptime and single race from the track)
  
5. Build the ROS2 wrappers:
    - open a windows ROS2 command and be sure to have ROS2 sourced
    - create a workspace
    - clone this repo
    - call install\setup.bat
    - ros2 run simulator_output assetto_corsa_handler.py
    - You should be able to see the published topics
  
      
