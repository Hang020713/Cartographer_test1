#!/bin/bash

sudo apt update
sudo apt install -y gpiod libgpiod-dev python3-libgpiod python3-pip python3-gpiozero python3-lgpio


sudo usermod -aG gpio $USER
