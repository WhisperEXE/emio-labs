::::::: collapse Kinematics of Emio
## Kinematics of Emio

In this section, we will explore the kinematic model of Emio, depending on the configurations of the legs.
After modeling each leg of the robot and the mechanical coupling between these legs (depending on the effector connector), 
we obtained the static calculations of the robot.
To observe the kinematics, you will test different configurations of Emio. 

As explained above, the kinematics of the robot can be expressed as a function that gives the position of the end-effector 
$\textcolor{darkgreen}{\mathbf{y}_{e}}$ based on the commanded motor position $\textcolor{red}{\mathbf{u}_{a}}$. 
To compute this function, we will modify the static force calculations, incorporate the coupling of the four legs, 
and impose the motor motion.

In the following, the indices letters $\mathbf{a}$ and $\mathbf{e}$ refer respectively to the actuation (motor) and the end-effector. 

In the algorithm, the four values of the actuation torque are introduced into the simulation as a vector of 
Lagrange multiplier $\boldsymbol{\lambda}_{\mathrm{a}}$. Furthermore, the motor positions in the simulation, 
$\boldsymbol{\delta}_{\mathrm{a}}$, is a function of the robot's position $\mathbf{q}$ 
(which is a concatenation of the leg positions: $\mathbf{q}_1,  ..., \mathbf{q}_4$).

![](assets/data/images/lab2-algorithm2.png){width=65%, .center}

To compute Equation 15 (in the algorithm above), it is more efficient to proceed with an indirect solution. 
We will decompose the movement at each time step by separating the contributions from the force $\mathbf{b}$, which is 
related to internal forces, external forces, and gravity (whose values we can compute), and the forces $\mathbf{H}_{\mathrm{a}}^T$ 
related to actuation (whose values are unknown and depend on the force required to satisfy the constraints).

$$
\mathbf{A}d\mathbf{q} = \mathbf{b} + \mathbf{H}_{\mathrm{a}}^T \boldsymbol{\lambda}_{\mathrm{a}} 
\Leftrightarrow
\left \{
\begin{array}{l}
d\mathbf{q} = d\mathbf{q}^{\mathrm{free}}  + d\mathbf{q}^{\lambda} \\ 
\mathrm{with}: \\
\mathbf{A}d\mathbf{q}^{\mathrm{free}} =  \mathbf{b} \leftrightarrow d\mathbf{q}^{\mathrm{free}} =  \mathbf{A}^{-1}\mathbf{b} \\
\mathbf{A}d\mathbf{q}^{\lambda} = \mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}} 
\leftrightarrow d\mathbf{q}^{\lambda}  = \mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}}
\end{array}
\right .
$$

Thus, we can rewrite the kinematic constraint, 
$\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q} = \textcolor{red}{\mathbf{u}_{a}}$, 
as directly depending on the actuation force:

$$
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}\left(d\mathbf{q}^{\mathrm{free}} + d\mathbf{q}^{\lambda}\right) = \textcolor{red}{\mathbf{u}_{a}} \Longleftrightarrow
$$

$$
\underbrace{
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}}}_{\boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}}} + 
\underbrace{\mathbf{H}_{\mathrm{a}}\mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T}_{\mathbf{W}_{\mathrm{aa}}} \boldsymbol{\lambda}_{\mathrm{a}} = \textcolor{red}{\mathbf{u}_{a}}
$$

This equation expresses the coupling of the actuation motion by the various torques via the compliance matrix $\mathbf{W}_{\mathrm{aa}}$, 
which represents the projection of the inverse matrix in the space of motor constraints.

The same way, we can rewrite 

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} = \boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{e}} d\mathbf{q} 
$$

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} =
\underbrace{
\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}}}_{\boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}}} +
\underbrace{\mathbf{H}_{\mathrm{e}}\mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T}_{\mathbf{W}_{\mathrm{ea}}} \boldsymbol{\lambda}_{\mathrm{a}}
$$

Combining equations above, we obtain a reduced formula of the linearized kinematics:

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} = \boldsymbol{\delta}_{\mathrm{e}}^{\mathrm{free}} + \mathbf{W}_{\mathrm{ea}}\mathbf{W}_{\mathrm{aa}}^{-1} ( \textcolor{red}{\mathbf{u}_{a}} - \boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}})
$$

$\mathbf{J}_{\mathbf{SR}} = \mathbf{W}_{\mathrm{ea}}\mathbf{W}_{\mathrm{aa}}^{-1}$ being the jacobian of the soft robot. 

:::::::
