# FOLEX
Four-Legged Experimental Robot

## REQUIREMENTS
* Ubuntu 18.04 or above
* CMake 3.15
* DYNAMIXEL SDK
* Eigen 3.x

## INSTALL DYNAMIXEL SDK
```
$ cd <your-dev-directory>
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd ~/DynamixelSDK/c++/build/linux_sbc
$ make -j$(nproc)
$ sudo make install
```

## INSTALL EIGEN 3.x
```
$ sudo apt install libeigen3-dev
```

## CLONE AND BUILD
```
$ cd <your-dev-directory>
$ git clone https://github.com/danichoi737/folex.git
$ cd folex
$ mkdir build && cd build
$ cmake ..
$ cmake --build .
```

## ADD USB RULES
```
$ cd scripts
$ chmod +x create_udev_rules
$ ./create_udev_rules
```
