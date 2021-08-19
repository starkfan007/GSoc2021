# GSoc2021
This is the summit work repository for GSoC2021.

## Background
Because image signal processing involves a large amount of data and strict real-time requirements, ISP usually adopts hardware implementation. making it difficult to customize the imaging algorithm for developers. Especially in certain scenarios, the default camera pipeline may not meet the imaging requirements, and we need to design better algorithms. So a software-based ISP would be useful for testing and experimentation.
## Install
To fetch the sources, build and install:
```
git clone git://linuxtv.org/libcamera.git
cd libcamera
meson build
ninja -C build install
```
Referring to [libcamera website](https://libcamera.org/getting-started.html), The dependent packages are required for building libcamera. Run `build/src/cam/cam -l` and list the camera available.

You can use the pipeline hanlder isp for tesing isp algorithm. First, you need to add corresponding file in this repository to corresponding folder in libcamera.
Then just following below command:
```
cd build
meson configure -Dpipelines=isp -Dipas= -Dtest=false
cd ..
ninja -C build
```

