#!/bin/bash
# PART 3: Python Environment Setup
# Usage: ./setup_python.sh (DO NOT RUN AS ROOT/SUDO)

echo "[INFO] Installing Miniconda..."
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh

echo "[INFO] Initializing Conda for Bash..."
~/miniconda3/bin/conda init bash
source ~/.bashrc

echo "[INFO] Creating 'physical_ai' Environment..."
# Python 3.10 is chosen for broad compatibility with Isaac Sim and SB3
conda create -n physical_ai python=3.10 -y

echo "[INFO] Activating Environment..."
# We must use the full path or 'eval' because the shell isn't restarted yet
eval "$(~/miniconda3/bin/conda shell.bash hook)"
conda activate physical_ai

echo "[INFO] Installing PyTorch (CUDA 12.1)..."
# We explicitly invoke the nvidia channel to get the correct CUDA runtime
conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia -y

echo "[INFO] Installing Reinforcement Learning Libraries..."
# Stable Baselines3: The standard for RL implementations
# Gymnasium: The standard for environment APIs
# WandB: For experiment tracking
pip install stable-baselines3[extra] gymnasium[all] wandb notebook

echo "[INFO] Environment Setup Complete. Type 'conda activate physical_ai' to start."
