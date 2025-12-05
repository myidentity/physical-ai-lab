# Physical AI Laboratory

A comprehensive open-science repository for Physical Artificial Intelligence, focusing on NVIDIA Isaac Sim, ROS2, and Reinforcement Learning.

## Repository Structure

- **`blog/`**: The Quarto-based documentation website. Contains executable notebooks and markdown posts.
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

### 2. Running the Blog Locally
This requires [Quarto](https://quarto.org/docs/get-started/) to be installed.

```bash
cd blog
quarto preview
```

### 3. Training an Agent
Scripts are located in `src/`.

```bash
python src/agents/train_ppo.py
```
