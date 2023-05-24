## Installation PX4/Gazebo

Works in Ubuntu 20.04

1. Installing PX4 Firmware - [Link](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

1. Restart PC
2. Install GCC

```bash
sudo apt-get install -y gcc-8-arm-linux-gnueabihf g++-8-arm-linux-gnueabihf
sudo update-alternatives --install /usr/bin/arm-linux-gnueabihf-gcc arm-linux-gnueabihf-gcc /usr/bin/arm-linux-gnueabihf-gcc-8 100 --slave /usr/bin/arm-linux-gnueabihf-g++ arm-linux-gnueabihf-g++ /usr/bin/arm-linux-gnueabihf-g++-8
sudo update-alternatives --config arm-linux-gnueabihf-gcc
```

4. Installing QGroundControl - [Link](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```

1. Logout and login again to enable the change to user permissions.
2. Download latest QGC release [HERE](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage), give permission and open it

```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
```

## HITL Set-up

PX4 HITL website - [Link](https://docs.px4.io/main/en/simulation/hitl.html)

1. Plug in the flight controller, open QGC, go to setup > safety and enable hitl
2. Change airframe to HITL Quadcopter and reboot Pixhawk
    1. If it doesn’t change you need to update the firmware from FMUv2 to FMUv3
    2. Flash the latest PX4 stable firmware, then go to parameters and set SYS_BL_UPDATE=1 and reboot Pixhawk - [Link](https://docs.px4.io/main/en/config/firmware.html)
    3. Flash again the latest version and now it should be FMUv3, then change airframe
3. In QGC Application settings, in the AutoConnect tab select only UDP (so that it will only connect to the simulator)

## Gazebo

1. Build PX4 with Gazebo (in order to build the Gazebo plugins)

```bash
cd <Firmware_clone> #usually /PX4-Autopilot folder
DONT_RUN=1 make px4_sitl_default gazebo
```

1. Set up the environment variables:

```bash
source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
```

1. Connect Pixhawk and run Gazebo in hitl mode

```bash
gazebo Tools/simulation/gazebo/sitl_gazebo/worlds/hitl_iris.world
```

1. Now open QGC and it will auto-connect through UDP to Gazebo. You can make the drone fly inside gazebo sending commands from QGC.

## Compile a Gazebo Plugin

Motors are defined as gazebo plugins in the drone model. A plugin contains a set of methods that define its behaviour, for example when the plugin receives some velocities it makes the rotors move and so on. In general, the plugin must be written in C++ and then compiled as binary which is then used by the simulator.

Gazebo API Documentation - [Link](https://classic.gazebosim.org/api)

- Drone model iris_hitl: PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models/iris_hitl/iris_hitl.sdf
- Motor model header: PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/include/gazebo_motor_model.h
- Plugins folder: PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/src
- CMakeList: PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/CMakeLists.txt

To modify and compile and existing plugin: 

1. Copy the plugin you want to modify in the same folder and rename it
2. Modify the C++ code, declare variables in the motor model header if possible
3. Add in the CMakeList file the new plugin reference to be compiled, for example

```makefile
add_library(gazebo_motor_model SHARED src/gazebo_motor_model_delay.cpp)
```

1. Build again the model (it will update only the modified files)

```bash
cd <Firmware_clone> #usually /PX4-Autopilot folder
DONT_RUN=1 make px4_sitl_default gazebo
```

1. Open Gazebo to see the changes (you can right click on the model > edit model to view all the installed plugins)

## Build Custom Firmware

Follow the steps [here](https://docs.px4.io/main/en/modules/hello_sky.html)