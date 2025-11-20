# Orbbec Lidar SDK

This SDK provides a comprehensive library for integrating and utilizing Orbbec Lidar devices, currently supporting the single-line lidar.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Dependencies](https://www.google.com/url?sa=E&source=gmail&q=#dependencies)
4. [Building the SDK](#building-the-sdk)
5. [Running Sample Applications](#running-sample-applications)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

* **Operating System**: Ubuntu 20.04 (recommended) or Ubuntu 18.04
* **Build System**: CMake (3.13 or higher)
* **Programming Language**: C++17
* **Point Cloud Processing**: Open3d (Point Cloud Library) for running provided examples

### Ubuntu 18.04 Users

Ubuntu 18.04 requires a CMake upgrade:

```bash
sudo apt install -y software-properties-common lsb-release wget gnupg2
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install cmake
sudo apt upgrade # if you have already installed cmake
```

**Note**: Ubuntu 18.04 can only compile the SDK. Running examples and tests is not supported.

## Installation

Install the necessary packages:

```bash
sudo apt-get install build-essential cmake libc++-dev libc++abi-dev
# if you want to run the provided examples
sudo add-apt-repository ppa:roehling/open3d
sudo apt-get update
sudo apt-get install libopen3d-dev
# or you can install Open3D from source
# please refer https://www.open3d.org/docs/release/compilation.html for more details
```

## Dependencies

* **eigen**: 3.4 (for examples) - [https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
* **mcap**: 1.4.0 - [https://github.com/foxglove/mcap/tree/main/cpp/mcap](https://github.com/foxglove/mcap/tree/main/cpp/mcap)
* **memory**: 0.7-3 - [https://github.com/foonathan/memory/releases/tag/v0.7-3](https://github.com/foonathan/memory/releases/tag/v0.7-3)
* **open3d**: 0.18.0 (for examples) - [https://github.com/isl-org/Open3D/releases/tag/v0.18.0](https://github.com/isl-org/Open3D/releases/tag/v0.18.0)
* **protobuf**: v21.12 - [https://github.com/protocolbuffers/protobuf/releases/tag/v21.12](https://github.com/protocolbuffers/protobuf/releases/tag/v21.12)
* **spdlog**: 1.14.1 - [https://github.com/gabime/spdlog/releases/tag/v1.14.1](https://github.com/gabime/spdlog/releases/tag/v1.14.1)
* **uvw**: v3.4.0_libuv_v1.48 - [https://github.com/skypjack/uvw/releases/tag/v3.4.0_libuv_v1.48](https://github.com/skypjack/uvw/releases/tag/v3.4.0_libuv_v1.48)
* **google test**: v1.15.2 (for testing) - [https://github.com/google/googletest/releases/tag/v1.15.2](https://github.com/google/googletest/releases/tag/v1.15.2)
* **mcap schema**: v0.2.1 - [https://github.com/foxglove/schemas/releases/tag/releases%2Fpython%2Ffoxglove-schemas-protobuf%2Fv0.2.1](https://github.com/foxglove/schemas/releases/tag/releases%2Fpython%2Ffoxglove-schemas-protobuf%2Fv0.2.1)

## Building the SDK

1. Clone the repository:

   ```bash
   git clone https://github.com/orbbec/orbbec_lidar_sdk.git
   cd orbbec_lidar_sdk
   git checkout develop
   ```
2. Create and navigate to the build directory:

   ```bash
   mkdir build && cd build
   ```
3. Configure the build with CMake:

   ```bash
   cmake .. -DBUILD_EXAMPLES=ON # Set to OFF for Ubuntu 18.04
   ```
4. Compile the code:

   ```bash
   make -j8
   make install
   ```

## Running Sample Applications

The following sample applications are available after building the SDK:
Blow command assume you are in the build directory

* **Scan Viewer**: Demonstrates basic SDK usage.

  ```bash
  cd install/bin
  ./scan_viewer
  ```
* **Discovery Devices**: Shows how to discover Orbbec Lidar devices.

  ```bash
   cd install/bin
  ./discovery_devices
  ```
* **Option Utils**: Demonstrates setting and getting device options.

  ```bash
    cd install/bin
  ./options/options
  ```
* **Record and Playback**: Records and plays back data from a device.

  ```bash
  cd install/bin
  ./recorder
  # To playback the recorded data
  ./playback point_cloud.mcap
  ```

## API Usage

Refer to the `examples` directory and the `src/option_utils.cpp` file for more detailed information on available options and API usage.

## Troubleshooting

If you encounter any issues:

1. Ensure all prerequisites are met.
2. Verify your operating system and CMake versions are compatible.
3. Use the latest SDK version from the GitHub repository.

For further assistance, please open an issue on the GitHub repository.
