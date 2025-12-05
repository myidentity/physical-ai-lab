#!/bin/bash
# PART 2: Docker & NVIDIA Container Toolkit
# Usage: sudo ./setup_docker.sh
# Run this AFTER rebooting from the driver installation.

set -e

echo "[INFO] Installing Docker Engine..."
# Uses the official convenience script from Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
rm get-docker.sh

echo "[INFO] Configuring NVIDIA Container Toolkit Repositories..."
# This is required to get the specific nvidia-ctk tools
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

apt-get update

echo "[INFO] Installing NVIDIA Container Toolkit..."
apt-get install -y nvidia-container-toolkit

echo "[INFO] Configuring Docker Daemon for GPU Support..."
# This modifies /etc/docker/daemon.json to enable the nvidia runtime
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

echo "[INFO] Verifying GPU Access in Docker..."
# This pulls a small container and runs nvidia-smi. 
# If this prints the GPU table, the setup is successful.
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi

echo "[INFO] Docker Setup Complete."
