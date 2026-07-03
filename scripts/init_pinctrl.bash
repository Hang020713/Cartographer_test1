# Ubuntu server 20.04 LTS aarch64 confirmed
#!/bin/bash

sudo apt update
sudo apt install -y cmake git device-tree-compiler build-essential libncurses5-dev libncursesw5-dev libfdt-dev

cd ~
git clone https://github.com/raspberrypi/utils.git
cd utils

cmake .
make
sudo make install

cd pinctrl
cmake .
make
sudo make install