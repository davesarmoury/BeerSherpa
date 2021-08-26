#!/usr/bin/env bash

sudo apt update
sudo apt install unzip wget

wget https://bin.equinox.io/c/4VmDzA7iaHb/ngrok-stable-linux-amd64.zip
unzip ngrok-stable-linux-amd64
rm ngrok-stable-linux-amd64.zip
