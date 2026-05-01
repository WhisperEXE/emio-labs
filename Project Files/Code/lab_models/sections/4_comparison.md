:::::: collapse Model Comparison: Beam vs. Volumetric FEM
## Model Comparison: Beam vs. Volumetric FEM

In previous sections, we treated the robot's legs using different mathematical approaches, ranging from standard 1D deformations to full 3D meshes. In soft robotics, choosing the right mechanical representation is a constant trade-off between **computational speed** and **physical accuracy**. 

Below is a theoretical comparison of the 1D topological models against the 3D volumetric Finite Element Method (FEM) models to help you understand how they predict the real-world bending and twisting of silicone structures.

### The 1D Models (Topological)
These models approximate the leg as a 1-dimensional line with thickness and width properties. They compute very fast, making them excellent for real-time control applications, but they can struggle to accurately represent complex 3D twisting or non-linear stretching.

* **Standard Beam (`beam`)**: Uses standard Hooke's Law. It is the baseline for fast linear calculations. It is highly efficient but assumes small deformations and linear material behavior.
* **Cosserat Rod (`cosserat`)**: Parameterized via rates of bending, torsion, and elongation. It is generally better at handling large, non-linear deformations in a 1D space than a standard beam, as it evaluates motion parameters in a local strain space.

### The 3D Volumetric Models (Tetrahedral FEM)
These models represent the leg as a full 3D mesh composed of tetrahedrons. They are computationally heavier but can capture the true geometry, volume preservation, and advanced material behaviors of the silicone.

* **Standard Tetra (`tetra`)**: A standard corotational volumetric FEM approach. It separates rigid deformations from purely elastic ones, allowing for the use of simple linear elasticity while handling large global rotations.
* **Tetra Linear (`tetra_linear`)**: Utilizes a grafted Direct Solver. Direct solvers can improve numerical stability during static equilibrium tasks and large deformations compared to iterative solvers, though they may require more memory.
* **Hyper-Elastic (`hyper`)**: Incorporates a Hyper-Elastic force field. This is highly relevant for soft robotics, as it mathematically accounts for the non-linear "stretching" behavior inherent to materials like silicone rubber, which do not obey Hooke's law under large strains.
* **Non-Uniform (`nonuniform`)**: Allows for varying stiffness (Young's Modulus) across different parts of the 3D mesh. This is particularly useful for simulating composite materials, biological tissues, or 3D-printed parts with varying infill densities.

### Trajectory Analysis and Visualization

To empirically compare these models, we can log the 3D coordinates of specific points of interest—such as the central grey mass (GM), and two tracking markers along the leg (M1, M2)—as the actuator moves through defined rotational intervals.

**1D Model Trajectories**
By plotting the spatial trajectories of the 1D models, we can observe the divergence in how each topological approach predicts the bending curve. Because the Cosserat formulation evaluates torsion and elongation differently than a standard beam, the resulting Z and Y coordinates at the tip (GM) will branch apart as the motor angle increases.

![](assets/labs/lab_models/beam_visualization.png){width=100% .center}

![](assets/labs/lab_models/gifs/blueleg.gif){width=75% .center}

**3D Volumetric Trajectories**
Similarly, the volumetric models display distinct kinematic paths under the exact same actuation. Plotting the Hyper-elastic, Non-Uniform, Tetra Linear, and Standard Tetra models reveals how advanced material formulations alter the predicted positions. For instance, the hyper-elastic model usually predicts a greater "sag" under gravity due to the non-linear yielding of the elements.

![](assets/labs/lab_models/volume_visualization.png){width=100% .center}

![](assets/labs/lab_models/gifs/whiteleg.gif){width=75% .center}


**Validating with Real-World Data**
While these simulated visualizer plots form the theoretical foundation for physical validation, comparing them directly against physical camera tracking data presents significant practical challenges. Precisely measuring the exact 3D coordinates of a highly deformable soft robot in a physical environment is notoriously difficult and prone to occlusion or sensor noise. Therefore, computing the **Mean Squared Error (MSE)** or **Mean Absolute Error (MAE)** between the simulated markers (M1, M2) and real-world camera coordinates is relegated to **future work**. Once robust physical tracking methods are established, calculating these error metrics will become the definitive way to determine which mathematical model offers the highest fidelity for a specific soft robot configuration.

### Key Considerations for Selection

When selecting a model for your soft robot, consider the following:

1.  **Real-time Requirements:** If the model must run in a high-frequency control loop (like an Inverse Kinematics controller), the 1D **Beam** or **Cosserat** models are often preferred due to their low computational cost.
2.  **Geometric Accuracy:** If the interaction involves complex collisions, self-intersections, or gripping objects where the surface geometry matters, a **3D Volumetric** model is necessary.
3.  **Material Behavior:** For highly extensible materials (like the white silicone legs under heavy load), standard linear models will fail to capture the true deformation curve. A **Hyper-Elastic** model will yield the most physically accurate results.
4.  **Mesh Resolution:** As seen in the volumetric section, the accuracy of a 3D model is heavily dependent on the mesh size. A coarse mesh will artificially stiffen the simulated object, requiring arbitrary tuning of the Young's Modulus to match reality.

By understanding these trade-offs, you can select the appropriate mathematical representation to balance the speed and fidelity needed for your specific robotic task.
::::::