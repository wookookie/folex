# FOLEX
Four-Legged Experimental Robot

## REQUIREMENTS
* Ubuntu 18.04
* CMake 3.15

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
