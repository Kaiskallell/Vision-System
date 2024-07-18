# Welcome to the Baumer neoAPI for C++

This package provides you with everything you need to program a Baumer camera with the Baumer neoAPI. If You are looking for a package for Python or Microsoft C#, please download it separately from the [Baumer web site](https://www.baumer.com/c/333)

## Prerequisites

- Download the package for your operating system and architecture [here](https://www.baumer.com/c/333)
- Have a suitable host system (64 bit Windows or Linux) or ARM board (AArch64 Linux) with at least one Gigabit Ethernet or USB3 port ready
- Have a Baumer GigE or USB camera including necessary cables and suitable power supply to play with
- Have a suitable C++ development environment of your choice (e.g. Visual Studio 2013 or greater, Eclipse with gcc 4.8.1 or greater)
- Optionally you can download and install the Baumer Camera Explorer, a graphical tool which will help you to understand and work with cameras
- Optionally install CMake (at least version 3.9.3) to build the examples
- Some provided examples require OpenCV (at least Version 3), if you want to build them you need to install it at a location of your choice

## Install the required drivers

- Windows
  - If USB cameras are used it is necessary to install the USB-driver provided with the package (see `/drivers/`)
  - For GigE cameras the Baumer filter-driver reduces the system-load compared to the Windows socket driver. We recommend installing and using the filter-driver provided with the package (see `/drivers/`)
  - For further information how to get the most performance out a GigE connection on a Windows system please refer to the application notes [AN201622](https://www.baumer.com/a/gigabit-ethernet-adapter-settings) and [AN201802](https://www.baumer.com/a/10-gige-and-baumer-gapi).
- Linux
  - If you are using Linux no driver needs to be installed.
  - To provide non root users access to usb devices copy the provided udev-rules (see `/drivers/udev_rules/`) in the drivers folder to `/etc/udev/rules.d/`.
  - Restart the system for the changed udev-rules to take effect
  - As the Linux kernel usually sets a 16 MB memory limit for the USB system, it might be necessary to raise the memory limit, especially for multi-camera systems. Please see the application note [AN201707](https://www.baumer.com/a/bgapi-sdk-for-usb-multi-camera-systems-with-linux) for details.
  - To prevent resend requests for lost pakets you should check udp and memory settings f.i. by `sysctl -a | grep mem`. To secure enough storage space add to /etc/sysctl.conf: 
~~~
### TUNING NETWORK PERFORMANCE ###
# Maximum Socket Receive Buffer
net.core.rmem_max = 67108864
~~~

> __Attention__
>
> If you have trouble connecting to a Baumer GigE camera, it might be necessary to configure the network settings of the camera first. You'll find the network configuration tool `gevipconfig` in the tools folder of this package.

## Try the examples

### Using the pre-compiled binaries

We provide all examples as binaries so you can check see the camera working without any further steps. The binaries are located in the /examples/bin folder.

### Using CMake to build the examples project

We also provide the CMakeList.txt files. CMake can use those to generate for many build-tools or projects for IDE's supported by CMake. We require at least CMake 3.9.3 to be used. The command `cmake --help` will show you all available generator options for your system. Please see the [CMake Generators Documentation](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html) for details. Some of our examples require OpenCV which you need to install separately if you want to build those examples.

__Attention__
Windows restricts the path to around 250 characters! Building the application using CMake creates many stacked folders which can lead to issues. If you get unexpected errors, try to work in a "short" folder like `C:\Baumer\`

Example using CMake (version 3.14) and the Visual Studio 2013 build tools:

Change to the root of your downloaded neoAPI package (where the CMakeLists.txt file is located) than run the CMake generate step. You need to provide the following options:

- The source directory (-S `<source_directory>`)
- The build directory (-B `<build_directory>`)
- The desired generator (-G <generator_string>), `cmake --help` gives you a list with provided generators
- Optional, the path to your installed OpenCV (-DOpenCV_DIR=<your_opencv_install_path>), this path should contain the `OpenCVConfig.cmake` file.

~~~bash
cmake.exe -S examples/src -B examples/build -G "Visual Studio 12 2013 Win64" -DOpenCV_DIR=your_opencv_install_path
~~~

Now run the CMake build step providing the build dir (--build `<build_directory>`) specified above.

~~~bash
cmake.exe --build examples/build
~~~

## License

Please see the /License file for licensing information.
