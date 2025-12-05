# Physical AI Laboratory

A comprehensive open-science repository for Physical Artificial Intelligence, focusing on NVIDIA Isaac Sim, ROS2, and Reinforcement Learning.

## Repository Structure

- **`blog/`**: Documentation website source. Contains tutorials and guides.
- **`src/`**: The source code for RL agents, custom environments, and utilities.
- **`models/`**: (Ignored) Local storage for trained model checkpoints.
- **`logs/`**: (Ignored) Local storage for TensorBoard/WandB logs.

## Getting Started

### 1. Environment Setup
Ensure you have the NVIDIA drivers installed (see blog posts).

```bash
conda env create -f environment.yml
conda activate physical_ai
```

### 2. Documentation

Visit the live documentation at: **https://myidentity.github.io/physical-ai-lab/**

### 3. Training an Agent
Scripts are located in `src/`.

```bash
python src/agents/train_ppo.py
```
