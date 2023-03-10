
# **From Shadows to Light: A Swarm Robotics Approach with Onboard Control for Source Seeking in Constrained Environments**

[Replay animations of the experiments can be found in this playlist](https://www.youtube.com/playlist?list=PLQXaE6NbHSe2d1Y2zXLiO3y00d4w9OrXR)

[Real flight videos of experiments can be found in this playlist](https://youtube.com/playlist?list=PLQXaE6NbHSe39z1X2zpjZD6f6mB9f40u9)

## Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

The firmware files needed for the experiments are located in ```cf-firmware-gradient-vu/examples/app_gradient```

Quick access [here](https://github.com/tugayalperen/IROS23gradfollower/tree/main/cf-firmware-gradient-vu/examples/app_gradient)

Please follow the official bitcraze documentation to build and flash new firmware in the crazyflie with the app layer. 
More information about the app layer can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/).

Once the new app is flashed in the crazyflies. Please refer to the main [mision control script]

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) by Bitcraze.


## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on Bitcraze website.

## Running Experiments with the Mission Control

The files for the mission planner are in ```mission-control``` directory.

In ```config.json``` are stored the parameters used for the experiments and they will be used to update the swarm parameters on-the-fly. 

The individual Crazyflie addresses needs to be specified in the main script. 

When running ```mc_client.py``` two windows will appear. One window will containt the real-time plotting of the experiment. The second one contains the GUI. 

To launch the experiment ```Press U``` To update the parameters in your Crazyflie swarm. ```Press T``` to takeoff the swarm. ```Press O``` to initiate the light sequence.

Note that the smart sockets uses require a personal key. You will need to configure your own smart lights. 
