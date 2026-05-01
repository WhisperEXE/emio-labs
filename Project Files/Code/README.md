# ME5415: Advanced Soft Robotics - Emio Project

This repository contains the laboratory files and external SOFA plugins developed for the ME5415 Continuum Soft Robot Control: Emio robot. 

The project focuses on a systematic comparison and integration of various modelling techniques (1D topological vs. 3D volumetric), Inverse Kinematics formulations (Integrated, Reduced Actuator-Space, and OIM-Inspired) and custom Pick-and-Place tasks with Emio robot.

## Repository Structure

* `external_plugins/`: Contains the required compiled SOFA plugins for advanced volumetric physics (e.g., HyperElastic, NonUniform, Direct Solvers).
* `lab_models/`: Contains the forward kinematics modeling lab, including automated data collection scripts for comparing model trajectories.
* `lab_inversekinematics/`: Contains the IK formulations and automated benchmarking scripts to evaluate tracking error, computation time and motor command stability.
* `pick_and_place/`: Contains the full pick and place pipeline with configurable model/solver combinations and evaluation.
---

## Setup Instructions

To run these labs locally, you must integrate the files from this repository into your existing `emio-labs` installation.

### Step 1: Install External Plugins
For the advanced 3D volumetric models (Hyper-Elastic and Non-Uniform) to load correctly, you must copy the provided plugins into your Emio Labs directory.
1. Copy the entire `external_plugins` folder from this repository.
2. Paste it directly into the root of your `emio-labs` directory (e.g., `~/emio-labs/external_plugins`).

### Step 2: Launch Emio Labs

Click the App icon of Emio Labs to launch it.

### Step 3: Configure the Labs

Once the Emio Labs UI is open, you need to manually link the new laboratory folders so they appear in your interactive notebook.

1. Look at the top toolbar and click on `Labs`.

2. Select `Configure Labs` from the dropdown menu.

3. In the configuration window, click the `Open Folder` button to browse your file system.

4. Navigate to where you cloned this repository and select the `lab_models` folder. 

5. Repeat this process to add the `lab_inversekinematics` and `pick_and_place` folders.