#!/bin/bash

# Create hotspot connection
sudo nmcli connection add \
  type wifi \
  con-name Hotspot \
  autoconnect yes \
  wifi.mode ap \
  wifi.ssid testcm5 \
  ipv4.method shared \
  ipv4.addresses 192.168.4.1/24

# Set WiFi password
sudo nmcli connection modify Hotspot \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "testcm5"

# Enable the hotspot
sudo nmcli connection up Hotspot

# Enable SSH service
sudo systemctl enable ssh
sudo systemctl start ssh