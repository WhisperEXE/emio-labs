::::: collapse Inverse Kinematics
## Inverse Kinematics

The goal of the inverse kinematics process is to find the inverse of the previously described relationship, i.e., to compute the motor command positions $\textcolor{red}{\mathbf{u}_{\mathrm{a}}} = \boldsymbol{f}^{-1}(\textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})$. This means determining the motor inputs that result in the desired end-effector position.
There are several challenges in solving this inverse problem:

- **Non-uniqueness of the inverse**: The robot's structure is deformable, and as a result, the inverse relationship $\boldsymbol{f}^{-1}$ is not unique. Different motor positions $\mathbf{u}_{\mathrm{a}}$ can lead to the same end-effector position $\mathbf{y}_{\mathrm{e}}$, depending on the deformation of the robot's legs.
- **Internal forces and lack of analytical model**: The robot's kinematic model is based on internal forces within the deformable structure, and there is no general analytical model for $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$. This makes it difficult to derive a closed-form expression for the inverse function, particularly because $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$ is highly nonlinear.
- **Nonlinearity of the system**: As mentioned, $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$ is a nonlinear function. Therefore, solving for the inverse kinematics requires setting up an optimization process that provides motor positions $\mathbf{u}_{\mathrm{a}}$ to minimize the distance with the desired end-effector position $\mathbf{y}_{\mathrm{e}}$.

To handle these challenges, we typically employ optimization techniques such as Quadratic Programming (QP) and the Optimization-based Inverse Method (OIM).

![](assets/data/images/lab2-algorithm3.png){width=65%, .center}

### Standard Quadratic Programming (QP)

With the indirect solving, the optimization presented in equation 20 (in the algorithm above) can be rewritten:

$$
\left \{
\begin{array}{l}
\mathbf{A}d\mathbf{q}^{\mathrm{free}}  =  \mathbf{b} \\
%%
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{min}
\frac{1}{2}(\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + 
\mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}} 
+ \mathbf{W}_{\mathrm{ea}} \lambda_{\mathrm{a}}  
- \textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})^2 \\
%%
\mathbf{A}d\mathbf{q}^{\lambda} = \mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}} 
\end{array}
\right .
$$

The advantage is that the optimization algorithm corresponds to convex optimization (i.e. Quadratic Programming - QP) 
on small matrices $\mathbf{W}_{\mathrm{ea}}$. If we develop the equation above, we obtain the objective function:

$$
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{min} \left(
\frac{1}{2} \lambda_{\mathrm{a}}^T \mathbf{W}_{\mathrm{ea}}^T \mathbf{W}_{\mathrm{ea}} \lambda_{\mathrm{a}}
+ \mathbf{W}_{\mathrm{ea}}^T(\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + 
\mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}}   
- \textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})\lambda_{\mathrm{a}}
\right)
$$

Remember that the relation between the motors' torque and displacement 
is given by $\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}} = \textcolor{red}{\mathbf{u}_{a}}$. Thus, to limit the course of the actuators we can add the following constraint to the QP:

$$
\textcolor{red}{\mathbf{u}_{min}} \le
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
\le \textcolor{red}{\mathbf{u}_{max}}
$$

and to constrain the actuators to a specific blocked position $\textcolor{red}{\mathbf{u}_{0}}$ we can add the following constraint:

$$
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
= \textcolor{red}{\mathbf{u}_{0}}
$$

In the case where the number of end-effectors is smaller than the number 
of actuators, we can have several solutions. To achieve a unique solution, we add a term $\epsilon \mathbf{W}_{\mathrm{aa}}$ to optimize the 
deformation energy [[Coevoet17]](https://inria.hal.science/hal-01649355/document).


### Optimization-based Inverse Method (OIM)

An advanced variation of the standard QP is the **Optimization-based Inverse Method (OIM)**. This approach utilizes a **Damped Least Squares** formulation. It is particularly useful for soft robots to handle numerical singularities (positions where the robot mathematically loses degrees of freedom) and to smooth the real-time tracking behavior.

The OIM formulation introduces damping and gain parameters to the objective function:

$$
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{\min} \left( \frac{1}{2} \lambda_{\mathrm{a}}^T (\mathbf{W}_{\mathrm{ea}}^T \mathbf{W}_{\mathrm{ea}} + \mu \mathbf{I}) \lambda_{\mathrm{a}} + \text{gain} \cdot \mathbf{W}_{\mathrm{ea}}^T (\boldsymbol{\delta}_{\mathrm{e}}^{\mathrm{free}} + \mathbf{q}_{\mathrm{e}} - \mathbf{q}_{\mathrm{t}}) \lambda_{\mathrm{a}} \right)
$$

Where the parameters function as follows:
* **Damping factor ($\mu$):** Adding $\mu \mathbf{I}$ prevents the calculation of "infinite" torques near mathematical singularities. A small value (e.g., $1e-5$) provides stability without heavily braking the robot's physical motion.
* **Gain:** This multiplier prioritizes rapid tracking, dictating how aggressively the end-effector attempts to close the distance error $(\mathbf{q}_{\mathrm{e}} - \mathbf{q}_{\mathrm{t}})$ to the target.
* **Minimal Regularization:** Similar to standard QP, a small weight is added to the deformation energy to prioritize target accuracy over minimizing motor effort.

In the subsequent sections, you will implement and test both of these numerical optimization methods to control the physical soft robot.
:::::