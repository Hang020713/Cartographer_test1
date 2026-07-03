#!/bin/bash

# install network-manager
sudo apt install -y network-manager

# Create hotspot connection
sudo nmcli connection add \
  type wifi \
  con-name Hotspot \
  autoconnect yes \
  wifi.mode ap \
  wifi.ssid testingcm5 \
  ipv4.method shared \
  ipv4.addresses 192.168.4.1/24

# Set WiFi password
sudo nmcli connection modify Hotspot \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "testingcm5"

# Enable the hotspot
sudo nmcli connection up Hotspot

# Enable SSH service (raspbian)
#sudo systemctl enable ssh
#sudo systemctl start ssh

# Enable SSH service (ubuntu)
# sudo apt install openssh-server -y
# sudo systemctl enable --now ssh
# sudo systemctl start ssh
# sudo ufw allow ssh
# sudo nano /etc/ssh/sshd_config
# PasswordAuthentication yes
# KbdInteractiveAuthentication yes
# sudo systemctl restart ssh