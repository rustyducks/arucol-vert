# Arucol Vert

An executable to find the position of aruco markers relative to another one. Also includes a camera calibration executable.

## Installation

1. Install dependencies (a valid c++ toolchain, cmake and opencv)

    ```bash
    sudo apt-get install build-essential cmake libopencv-dev
    ```

2. Clone and compile

    ```bash
    git clone https://github.com/rustyducks/arucol-vert.git
    cd arucol-vert && mkdir build && cd build
    cmake ..
    make -j8
    ```

## Usage

* For camera calibration:
    1. Print an opencv chessboard.
    2. Update the config file in config/calibration_params.xml
    3. Run the executable `./arucol_vert_calib ../config/calibration_params.xml`
    4. Follow the instructions (press g and move the chessboard in front of the camera)
    5. Wait for it. It creates a file under config/out_camera_data.xml (as defined in calibration_params.xml).

* For normal use:
    1. Place the camera in a stable position. Make sure it can view the central marker.
    2. Run the executable `./arucol_vert CAMERA_CONFIG_FILE GENERAL_CONFIG_FILE [WITH_DISPLAY=0]`.
        (eg. `./arucol_vert ../config/out_camera_data.xml ../config/general_params.xml 1`)
    3. Enjoy. Note: the detection will only start if the central marker has been seen at least one.
