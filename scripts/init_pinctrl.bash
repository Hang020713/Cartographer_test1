#!/bin/bash

sudo apt update
sudo apt install -y cmake git device-tree-compiler

cd ~
git clone https://github.com/raspberrypi/utils.git
cd utils

cmake .
make
sudo make install

cd utils/pinctrl
cmake .
make
sudo make install