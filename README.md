# Autonomous Bin Picking (Deployed)

This repository contains an autonomous bin-picking project built using **RLBench**, **PyRep (CoppeliaSim)**, and **GQCNN**.
Due to the age of the original dependencies (~2021), the environment setup requires specific older versions of libraries and Python 3.6 to function correctly.

## 🚀 Quick Start: Hardcoded Grasping Demo
If you have completed the environment setup, you can run the hardcoded grasping demo to see the robot in action (bypassing the missing GQCNN models):

```bash
cd Autonomous-Bin-Picking
python demo_grasp.py
```
*This script will launch the RLBench environment, find an object, pick it up, drop it in the red container, and save screenshots (`scene_before.png` and `wrist_grasping.png`).*

---

## 🛠 Detailed Installation Guide

The installation is complex and must be done in a **Conda Python 3.6** environment.

### 1. Prerequisites
You need **Conda** and **Ubuntu 20.04/22.04**.

Create a new isolated environment:
```bash
conda create -n bin_picking python=3.6
conda activate bin_picking
```

### 2. Install CoppeliaSim (PyRep Dependency)
PyRep requires CoppeliaSim V4.1.0 to function.

```bash
# Download and extract CoppeliaSim V4.1.0 Ubuntu 20.04
wget -qO- https://downloads.coppeliarobotics.com/V4_1_0/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz | tar -xJ

# Set Environment Variables (Add these to your ~/.bashrc or a setup script)
export COPPELIASIM_ROOT=$(pwd)/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```

### 3. Clone Compatible Versions of Dependencies
Since the original project was built around 2020-2021, you MUST use older commits of the dependencies.

```bash
# Clone PyRep and RLBench
git clone https://github.com/stepjam/PyRep.git
git clone https://github.com/stepjam/RLBench.git

# Clone GQCNN and Perception
git clone https://github.com/BerkeleyAutomation/gqcnn.git
git clone https://github.com/BerkeleyAutomation/perception.git

# Checkout compatible versions
cd RLBench && git checkout 1.1.0 && cd ..
cd gqcnn && git checkout `git log --before="2020-12-31" -n 1 --format="%H"` && cd ..
cd perception && git checkout `git log --before="2020-12-31" -n 1 --format="%H"` && cd ..
```

### 4. Install Dependencies
Install them in the exact order below:

```bash
# Install PyRep
cd PyRep
pip install .

# Install RLBench
cd ../RLBench
pip install .

# Install GQCNN & Perception
cd ../gqcnn
pip install .
cd ../perception
pip install -e .
```

### 5. Fix Dependency Conflicts (Important)
Python 3.6 is deprecated, so modern pip wheels will fail to build (especially OpenCV and PyTorch). Run these commands to install compatible pre-compiled wheels:

```bash
# Fix OpenCV and PyTorch for Python 3.6
pip uninstall -y opencv-python
pip install opencv-python-headless==4.3.0.38
pip install torch==1.10.2+cpu torchvision==0.11.3+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html
pip install autolab_core
```

### 6. Install Project Requirements
Finally, install the requirements for this specific repo:
```bash
cd Autonomous-Bin-Picking
pip install -r requirements.txt
```

---

## 🛑 Known Issues
- **GQCNN Models Unavailable**: The original `main.py` relies on pre-trained GQCNN weights hosted on Berkeley's Box servers. Those links are currently dead (returning 404). Therefore, `main.py` will crash looking for `models/GQCNN-2.0/config.json`. To run it, you must manually source and place the GQCNN-2.0 weights in the `models/` directory. Use `demo_grasp.py` instead for testing.
- **Qt GUI Warnings**: If you see `QObject::~QObject: Timers cannot be stopped from another thread` at exit, it's a harmless Qt garbage collection warning and can be ignored (we bypass it in `demo_grasp.py` using `os._exit(0)`).
