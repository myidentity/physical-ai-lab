#!/bin/bash
# PART 1: System Prep & NVIDIA Drivers
# Usage: sudo ./setup_drivers.sh
# Purpose: Prepares a fresh Ubuntu 22.04 install for Deep Learning

set -e # Exit immediately if a command exits with a non-zero status

echo "[INFO] Updating Apt Repositories and Upgrading System..."
apt-get update && apt-get full-upgrade -y

echo "[INFO] Installing Essential Build Tools..."
# These libraries are required to build Python packages from source
# libgl1-mesa-dev etc. are required for rendering in headless environments
apt-get install -y build-essential cmake git unzip zip curl wget \
    libgl1-mesa-dev libgl1-mesa-glx libglew-dev libosmesa6-dev \
    software-properties-common libopenmpi-dev zlib1g-dev

echo "[INFO] Adding NVIDIA Graphics Drivers PPA..."
# The PPA provides access to the proprietary drivers required for CUDA
add-apt-repository ppa:graphics-drivers/ppa -y
apt-get update

echo "[INFO] Identifying Recommended Driver..."
# ubuntu-drivers tool helps avoid version conflicts
ubuntu-drivers devices

echo "[INFO] Installing NVIDIA Driver..."
# CRITICAL NOTE: Isaac Sim 4.x often requires specific driver branches.
# As of late 2024, the 535 or 550 branch is often recommended over the bleeding edge.
# We explicitly install a stable production branch to ensure compatibility.
apt-get install -y nvidia-driver-535

echo "[INFO] Driver Installation Complete."
echo " A SYSTEM REBOOT IS REQUIRED before proceeding."
