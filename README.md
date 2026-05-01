# Precision Pick-and-Place with EMIO Continuum Parallel Robot

This repository contains the framework, findings, and experimental results from our soft robotics project conducted at the **National University of Singapore (NUS)**. Our research focuses on the kinematic formulations and task-level control of the **EMIO robot**, a four-legged continuum parallel robot developed by **Compliance Robotics**.

## Project Overview
Soft robotic systems offer inherent compliance, making them ideal for delicate tasks like food handling. However, their infinite degrees of freedom make solving forward and inverse kinematics significantly more challenging than for rigid systems. 

This project presents a systematic study comparing different mathematical methods for computing kinematics and evaluates their performance in a high-precision environment. Detailed methodology and full analysis can be found in the attached **Report.pdf**.

## Task: Precision Pick-and-Place
Following the guidelines in the **Rule Book for the Design Project.pdf**, we implemented a structured pick-and-place task using the EMIO kit's 25 mm cube and a tray with cubic containers. The project involved:

* **Forward Kinematics Modeling**: Evaluation of six models including reduced-order rod approximations (Beam and Cosserat) and four volumetric FEM variants.
* **Inverse Kinematics (IK) Formulations**: Comparison of three optimization approaches (Integrated, Reduced Actuator-Space, and OIM-Inspired) solved as Quadratic Programs (QP).
* **Key Findings**: The **Beam model combined with the Integrated Inverse solver** emerged as the most suitable combination, achieving a 100% success rate with smooth convergence and low motor inputs.

## Implementation
The control system was implemented as a **Finite State Machine (FSM)** with 11 states, including pre-grasping, approaching, and releasing sequences. The system uses an open-loop architecture to evaluate the performance of different model/solver combinations in isolation.

## Acknowledgments
This project was developed for the **ME5415 Design Project** at the **National University of Singapore**. We would like to thank our coordinators, **Prof. Cecilia Laschi** and **Dr. Hongying Zhang**, for their guidance.

This work was a collaborative effort by:
* **Markus Grooss Karevold**
* **Vishal Mangla**
* **Kho Kah Chun**
* **Shridhar Ganesh Iyer**

---
*Note: This design project was developed with reference to the RoboSoft Competition 2023 guidelines and innovations.*
