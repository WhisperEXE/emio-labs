:::::: collapse Quadratic Programming (QP)
## Quadratic Programming (QP)

In this section we propose to observe again the behavior of the *white leg*.
With a new configuration of the robot and using the solver provided by SOFA to solve its inverse kinematics, 
you will compare the models and conclude on the advantages and disadvantages of each approach. 

### Understanding the Model Variants

* **Beam & Cosserat**: 1D formulations that are computationally fast but may lose accuracy during complex 3D rotations.
* **Tetra (Normal)**: Standard linear volumetric FEM.
* **Tetra Linear (Direct)**: Uses the grafted **Direct Solver** for improved numerical stability during the inverse kinematics optimization.
* **Hyper-Elastic**: Better models the non-linear "stretching" of the silicone legs by accounting for large strains.
* **Non-Uniform**: Accounts for varying stiffness across the leg geometry.


::::: exercise

::: collapse {open} Set up Emio 

Take four *blue legs* and put them on each motor as shown on the image.
Next, attach again the <span style="color:blue">*blue connector*</span> at the tip of each leg, and place
one <span style="color:green">*green marker*</span> on the top of the connector.

![](assets/data/images/lab2-exercice2-emio.png){width=75% .center}
:::

**Exercise 1: Blue Legs Implementation**

Try the various models with this setup of Emio. Move the effector target in the *x* direction. 

:::: select exo1model
::: option beam
::: option cosserat
::: option tetra
::: option tetra_linear
::: option hyper
::: option nonuniform
::::

#runsofa-button("assets/labs/lab_inversekinematics/qp_inversekinematics.py" "--legsName" "blueleg" "--legsModel" "exo1model" "--legsPositionOnMotor" "counterclockwisedown" "clockwisedown" "counterclockwisedown" "clockwisedown" "--centerPartName" "bluepart")

:::::


::::: exercise

::: collapse {open} Set up Emio 

Take four *white legs* and put them on each motor as shown on the image.
Next, attach again the <span style="color:blue">*blue connector*</span> at the tip of each leg, and place
one <span style="color:green">*green marker*</span> on the top of the connector.

![](assets/data/images/lab2-exercice3-emio.png){width=75% .center}
:::

**Exercise 2: White Leg Implementation**

Try the various models with this setup of Emio. Move the effector target in the *x* direction. 

:::: select exo2model
::: option beam
::: option cosserat
::: option tetra
::: option tetra_linear
::: option hyper
::: option nonuniform
::::

#runsofa-button("assets/labs/lab_inversekinematics/qp_inversekinematics.py" "--legsName" "whiteleg" "--legsModel" "exo2model" "--legsPositionOnMotor" "counterclockwisedown" "clockwisedown" "counterclockwisedown" "clockwisedown" "--centerPartName" "bluepart")

:::::

::::::