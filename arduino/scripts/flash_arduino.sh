# This script flashes the arduino due

BASE_DIR="barc_hardware_interface"

if [ ! -d ~/$BASE_DIR/arduino/.arduino_node ]; then
    mkdir -p ~/$BASE_DIR/arduino/.arduino_node/src
    mkdir -p ~/$BASE_DIR/arduino/.arduino_node/lib
fi

if [ ! -L ~/$BASE_DIR/arduino/.arduino_node/lib ]; then
    ln -s ~/arduino/sketchbook/libraries ~/$BASE_DIR/arduino/.arduino_node/lib
fi

cd ~/$BASE_DIR/arduino/.arduino_node
cp ../arduino_node/BARC_NODE_v4.ino src/;

ano clean; ano build -m mega2560;
ano upload -m mega2560 -p /dev/ttyACM0;
cd -
