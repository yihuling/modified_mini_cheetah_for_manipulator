## Cheetah-Software
### We add the jump motion part and draw plot part.
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified.

## Build
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j8
```

If you are building code on your computer that you would like to copy over to the mini cheetah real robot, you must replace the cmake command with
```
cmake -DMINI_CHEETAH_BUILD=TRUE
```
otherwise it will not work.  If you are building mini cheetah code one the mini cheetah computer, you do not need to do this.

This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j8` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/MIT_controller/mit_ctrl m s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot
## Draw the Debug Figure
1. Run the draw_real_data.py python script
   ![contents](https://github.com/CUHKSiriusLeggedRobotTeam/Cheetah_Software_24Nm_selfIMU/blob/lz_code/fig/Figure_3.png)
## Run Mini cheetah in real robot
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh` 


## Dependencies:
```
1. sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
sudo apt-get update sudo apt-get install openjdk-8-jdk
2. Install LCM(cd in LCM 1.3.1 (1)./configure (2)make (3) sudo make install (4) sudo ldconfig)
3. Install eigen(mkdir build (3) cd build (4) cmake .. (5) make install)
4. Install Qt5.12(chmod a+x qt-opensource-linux-x64-5.12.0.run)
To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..
```


## Setting Real Robot Configuration (upboard kernel)
```
1. sudo apt-get install libncurses5-dev sudo apt-get install libssl-dev 
2. cp .config kernel cd kernel make menuconfig 
3. make -j4 sudo make modules_install -j4 sudo make install -j4 sudo update-grub 
4. reboot
```

## If USE Ubuntu18.04 you should do this to fix ``` ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.22’ not found```
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9
sudo apt-get upgrade libstdc++6
```

### Test IS OK
```
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCX
```
