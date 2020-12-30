# barc_hardware_interface
ROS2 package for interfacing with the BARC hardware. Also includes BARC firmware.

## Setup instructions for the onboard Arduino DUE
Control of the BARC is performed at two levels. At the high level, the Jetson Nano is a part of the ROS network and, in the simplest case, only runs the Arduino interface node (`nodes/py_arduino_interface_node.py`) contained in this repository. (Perception, estimation, and control nodes may also be run on the Jetson, but is not relevant to these instructions). The role of the Arduino interface node is to subscribe to the desired control signals for steering angle and acceleration, convert them to PWM values for the steering servo and ESC, and send these values to the onboard Arduino, which performs low level actuation control, via serial communication.

These instructions detail how to flash the script which handles serial communication and low level control onto the Arduino DUE. All of the following steps should be performed on the Jetson Nano.

### Requirements
- For the ROS node, `pyserial` is required, which can be installed using `sudo python3 -m pip install pyserial`

### 1. Install the Arduino IDE
For the flashing Arduino, the Arduino IDE is required. This can be obtained by downloading the **Linux ARM 64 bits** version of the IDE from https://www.arduino.cc/en/software. Instruction for installing can be found [here](https://www.arduino.cc/en/Guide/Linux).

### 2. Install the core for the Arduino DUE
By default, the Arduino IDE does not include the core required for interfacing with the Arduino DUE and the official version is not compiled for the ARM 64 architecture. In order to work around this issue, we use the steps found [here](https://forum.arduino.cc/index.php?topic=572898.0)
- In the IDE, navigate to **File > Preferences** and add the URL: https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json in the dialog box labeled **Additional Boards Manager URLs**
- Navigate to **Tools > Board > Boards Manager...** and look for **Arduino SAM Boards (Arduino SAM Boards (32-bits ARM Cortex-M3) with ARM64 support** by **per1234**
- Click **Install** and wait for the installation to finish (this may take a while on the Jetson)
- After the installation is complete, the DUE should show up in **Tools > Board > Arduino ARM(32-bits) Boards**
- Choose the **Arduino Due (Native USB Port)** with port **/dev/tty/ACMx** (make sure that you grant write permission with `sudo chmod 777 /dev/tty/ACMx`)
- Verify that you can upload a sketch to the DUE

<!-- ### 3. Install the Arduino command line utility
- The script for performing the installation is located in `arduino/scripts/install_arduino_cmd.sh`
- The script will clone the repository containing the utility and install it  -->
