# Precision Pick-and-Place with EMIO Continuum Parallel Robot

This repository contains the framework, findings, and experimental results from our soft robotics project conducted at the **National University of Singapore (NUS)**. Our research focuses on the kinematic formulations and task-level control of the **EMIO robot**, a four-legged continuum parallel robot developed by **Compliance Robotics**.

## Project Overview
[cite_start]Soft robotic systems offer inherent compliance, making them ideal for delicate tasks like food handling[cite: 441, 691, 692]. [cite_start]However, their infinite degrees of freedom make solving forward and inverse kinematics significantly more challenging than for rigid systems[cite: 442, 450]. 

[cite_start]This project presents a systematic study comparing different mathematical methods for computing kinematics and evaluates their performance in a high-precision environment[cite: 443, 447, 448].

## Task: Precision Pick-and-Place
[cite_start]We implemented a structured pick-and-place task using the EMIO kit's 25 mm cube and a tray with cubic containers[cite: 694, 695]. The project involved:
* [cite_start]**Forward Kinematics Modeling**: Evaluation of six models including reduced-order rod approximations (Beam and Cosserat) and four volumetric FEM variants[cite: 445, 452, 456].
* [cite_start]**Inverse Kinematics (IK) Formulations**: Comparison of three optimization approaches (Integrated, Reduced Actuator-Space, and OIM-Inspired) solved as Quadratic Programs (QP)[cite: 446, 637].
* [cite_start]**Key Findings**: The **Beam model combined with the Integrated Inverse solver** emerged as the most suitable combination, achieving a 100% success rate with smooth convergence and low motor inputs[cite: 797, 779].

## Implementation
[cite_start]The control system was implemented as a **Finite State Machine (FSM)** with 11 states, including pre-grasping, approaching, and releasing sequences[cite: 701, 716]. [cite_start]The system uses an open-loop architecture to evaluate the performance of different model/solver combinations in isolation[cite: 704].

## Acknowledgments
[cite_start]This project was developed for the **ME5415 Design Project** at the **National University of Singapore**[cite: 403, 407]. We would like to thank our coordinators, **Prof. Cecilia Laschi** and **Dr. [cite_start]Hongying Zhang**, for their guidance[cite: 404].

This work was a collaborative effort by:
* **Markus Grooss Karevold** [cite: 439]
* [cite_start]**Vishal Mangla** [cite: 439]
* [cite_start]**Kho Kah Chun** [cite: 439]
* **Shridhar Ganesh Iyer** [cite: 439]

---
*Note: This design project was developed with reference to the RoboSoft Competition 2023 guidelines and innovations[cite: 406, 435].*
