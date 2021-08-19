# GSoc2021
This is the summit work repository for GSoc2021, including the code and patches. In short, I need to implement the software ISP in libcamera. For more details, you can refer my [blog](https://starkfan007.github.io/Gsoc-summit-work/).

## Hardware
So far, only three cameras officially supported by the Raspberry Pi can use libcamera to get Bayer format. They are ov5647, imx219, and imx477. Maybe other CSI cameras can also do it, but I haven't tested. Therefore, in order to run this, you need a Raspberry Pi 3/4 and a matching camera.

## Install
Referring to [here](https://libcamera.org/getting-started.html), The dependent packages are required for building libcamera.   
Then you can fetch the sources, build and install:
```
git clone git://linuxtv.org/libcamera.git
cd libcamera
meson build
ninja -C build install
```
 Run `build/src/cam/cam -l` and test the camera available.

You can use the pipeline hanlder isp for tesing isp algorithm. First, you need to add corresponding file in this repository to corresponding folder in libcamera.
Then just following below command:
```
cd build
meson configure -Dpipelines=isp -Dipas= -Dtest=false
cd ..
ninja -C build
build/src/cam/cam -c 1 -C[num]
```

