# VU Light Following Swarm for Crazyflie
Application with Crayflie app-Layer.

------
#### This directory is directly related to the following paper:
### 	From Shadows to Light: A Swarm Robotics Approach with Onboard Control for Seeking Dynamic Sources in Constrained Environments*


### Citation:
```
Coming soon
```
---
REQUIREMENTS
------------

Mission control software **Needs update**

Same requirements as Crazyflie firmware

## Installation
- clone the repository
```bash
git clone https://github.com/RetamalVictor/cf-firmware-gradient-vu.git
```

## Build

Make sure that you are in the app_swarm (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```
