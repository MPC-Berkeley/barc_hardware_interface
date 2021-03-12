# This script flashes the arduino due

# Make sure that this path is points correctly to the files related to the Arduino IDE
ARDUINO_DIR="/home/mpclab/arduino-1.8.13"
ARDUINO_PORT="/dev/ttyACM0"

# Create symbolic link
if [ ! -L "/usr/local/share/arduino" ] || [ ! -d "/usr/local/share/arduino" ]; then
   sudo ln -s ${ARDUINO_DIR} /usr/local/share/arduino
fi

BASE_DIR="barc_hardware_interface"

if [ ! -d ~/$BASE_DIR/arduino/.arduino_node ]; then
    mkdir -p ~/$BASE_DIR/arduino/.arduino_node/src
    mkdir -p ~/$BASE_DIR/arduino/.arduino_node/lib
fi

LIB_DIR="/home/mpclab/$BASE_DIR/arduino/.arduino_node/lib"
if [ ! -L ${LIB_DIR} ] || [ ! -d ${LIB_DIR} ]; then
    ln -s ${ARDUINO_DIR}/libraries ${LIB_DIR}
fi

cd ~/$BASE_DIR/arduino/.arduino_node
cp ../BARC_NODE_V4.ino src/;

sudo chmod 777 ${ARDUINO_PORT}

ano clean
ano build -m arduino_due_x 
ano upload -m arduino_due_x -p ${ARDUINO_PORT}

cd -
