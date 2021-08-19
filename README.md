# GSoc2021
This is the summit work repository for GSoC2021, including the code and patches. For more details, you can refer my [blog](https://starkfan007.github.io/Gsoc-summit-work/).

## Hardware
So far, only three cameras officially supported by the Raspberry Pi can use libcamera to get Bayer format. They are ov5647, imx219, and imx477. Maybe other CSI cameras can also do it, but I haven't tested. Therefore, in order to run this, you need a Raspberry Pi 3/4 and a matching camera.

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

