:::::: collapse Corotational Volume FEM Model

## Corotational Volume FEM Model

The corotational model aims to separate rigid deformations from purely elastic deformations. 
It is particularly useful for simulations where objects undergo significant rotations but the elastic 
deformations are relatively small. The corotational model relies on a principle of deformation separation: 
the total displacement of an element is decomposed into a rigid rotation followed by a purely elastic 
deformation. For the rigid rotation, the rotational part of the nodal displacements of a tetrahedron
is extracted using a technique such as Singular Value Decomposition (SVD) to obtain the rotation matrix. 
For elastic deformations, once the rotation is extracted, the remaining deformations are treated as small 
deformations within the framework of linear elasticity in a 'local' frame associated with each element. 
Finally, the contributions of the individual elements are assembled into the global system of the equation
of motion.

The corotational model allows for the use of a simple linear elasticity formulation while handling large
rotations. It is computationally efficient because it avoids the complex calculations required in other 
full nonlinear approaches. In the context of soft robotics, it is useful for managing significant rotations 
of moving structures that undergo relatively small elastic deformations.

## Hands-on

Now, you will experiment with the volume model and compare it to the beam and Cosserat models using a new leg (the white one). 
This exercise will help you understand the advantages and disadvantages of each method, as well as determine 
the best scenarios for their application.

Note that, similar to many other numerical methods, with FEM, there are concerns about the quality of the discretization, 
in addition to the resolution algorithm itself. The principle is that as the mesh becomes finer, 
the solutions should converge to the solution of the original partial differential equation.
Note that it is common to use a smaller value for the Young's modulus when using a coarse mesh.
See the below image for an example of the influence of the mesh resolution on the solution.
                                                          
|                                                                        ![](assets/data/images/volumemodel.png){width=75%}                                                                                                                                                                                                           |                                                                                                                            
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| **In this simulation the leg is attached at its base and is subjected to gravity. The illustration shows a comparison between the beam model (represented by frames) and the volume model (the blue meshes) with different mesh size factor: (a) 114 nodes, (b) 252 nodes, (c) 1932 nodes, (d) 11008 nodes, (e) 41692 nodes.** |

::::: exercise

:::: collapse {open} Set up Emio 

Take the *white leg* and put it on the *motor n°0* (clockwise down as shown on the image). 
Next, attach the <span style="color:grey">*grey cube*</span> at the tip of the leg, then place
two <span style="color:green">*green markers*</span> on the leg: one in the middle of the leg and the second at the tip,
just above the cube. 

![](assets/data/images/lab1-exercice2-leg.png){width=50% .center}

::::

**Script 1: Interactive Motor Control **

Run the following simulation and observe the behavior of the leg.

:::: select modelsexo2
::: option tetra 
::: option tetra_linear
::: option hyper
::: option nonuniform
::::

### Comparison of Volume Techniques

* **Normal Tetra**: Uses the standard internal corotational FEM.
* **Linear (Direct)**: Uses the grafted **Direct Solver** for better numerical stability during large deformations.
* **Hyper-Elastic**: Accounts for non-linear material stretching, ideal for soft silicone.
* **Non-Uniform**: Allows for varying stiffness across the mesh, useful for complex 3D-printed parts.

**Script 1: Interactive Motor Control**

Launch the simulation, make sure that the markers you put on the leg match the 
markers in the simulation, then change the motor's position. Observe the deformation differences 
between the simulation and the real leg. 

#runsofa-button("assets/labs/lab_models/lab_models.py", "whiteleg", "modelsexo2")

**Script 2: Predefined Motor Control with Data Logger**

Launch the simulation, the motor should move through 10 intervals, add 3 visualizers per interval and save the positions into a .csv file.

#runsofa-button("assets/labs/lab_models/lab_models_datalogger.py", "whiteleg", "modelsexo2")

:::::
::::::