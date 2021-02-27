# Moving_Base_Robotic_Manipulator_Dynamic_Model
Euler Lagrange based dynamic model for a robotic manipulator on a moving base.
This dynamic model has been developed on the basis of the journal C.M. Wronka and M.W. Dunnigan, "Derivation and analysis of a dynamic model of a robotic manipulator on a moving base", *Robotic and Autonomus Systems*, vol. 59, pp. 758-769, June. 2011, doi: 10.1016/j.robot.2011.05.010.

The need to create this dynamic model came while I was trying to match the results of the Euler Lagrange formulation code with the Recursive Newton Euler formulation (RNEA) code for the case of human leg dynamics (single leg) where the hip joint is considered the base. While it is fairly simple and intuitive to including motion of the hip (base) in the RNEA code, it is extremely challenging in the Euler Lagrange one, since the formulation itself has been developed on the back of some rigourous mathematics. The original code for the [Euler Lagrange based Dynamic Model](https://github.com/average-engineer/InvDynamics_Robotic_Manipulator_Euler-Lagrange_Formulation) is for the case of a **fixed** base manipulator, and thus needs to be modified in order to account for base movement.

The main assumptions of this dynamic model are:
- Base inertia and mass is significantly higher than the manipulator itself i.e. the manipulator motion doesn't effect the base movement in any way.
- Base Trajectory and its derivatives are considered as model parameters rather than generalized coordinates i.e. if we create a multi-body system out of the manipulator, then the base trajectory won't be counted as an independent system degree of freedom, rather it will be considered in the system matrix in the system's state space representation (`w_dot = A*w + B, A: System Matrix, w: State vector/generalized coordinates of system)`

The code structure is given below:

![Code Structure](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/EulerLagrangeDynamicModel_MovingBase.PNG)

Here, only the newly added function scripts for the new matrices have been discussed. For the other function scripts, refer to [Euler Lagrange based Dynamic Model](https://github.com/average-engineer/InvDynamics_Robotic_Manipulator_Euler-Lagrange_Formulation) in my profile.


1). `inertial_force_base.m` computes the matrix representing the effect of the inertial acceleration of the base on the manipulator links. It is dependent on the base coordinates, velocity and acceleration and independent of the system generalized coordinates. It is a `nx1` matrix.

2). `centri_cor_base_matrix.m` computes the matrix representing the effect of the centrifugal and coroilis force on the manipulator due to base movement. It is dependent on the base coordinates, velocity and acceleration and independent of the system generalized coordinates. It is a `nxn` matrix and multiplies with the system generalized coordinate matrix.

3). `gravity_loading_matrix.m` is a function which calculates the gravity loading matrix, whose each term corresponds to the potential energy considerations of each link due to gravity. In the case of moving base, it depends on the base position coordinates.


## Validating Dynamic Model
For validation, the simple example of a 1-D cart pole system is considered. This system is extremely simple, with the cart being a moving base and the pole can be considered as a single link manipulator. The pole is connected to the cart with the help of a revolute joint. 

The anaytical solution of the cart pole system has been derived using tradiational Euler-Lagrange method after which the hard code is developed for the analytical expression, from which the anaytical solution for pole joint torque is computed. The analytical hard code was compiled in a MATLAB Live Script, which can be accessed from this repository, but for illustration purposes, the snippets of the live script have been shared here:

![LiveScriptSnippet](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/Live_Script_Snippet1.PNG)


![LiveScriptSnippet](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/Live_Script_Snippet2.PNG)


![LiveScriptSnippet](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/Live_Script_Snippet3.PNG)


![LiveScriptSnippet](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/cart_trajectory.png)


Since this cart pole system can be seen as a 1 link manipulator with a moving base, the [Moving Base Dynamic Model](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model) is modified accordingly. **Important thing to note is that for the dynamic model, the cart (base) movement isn't considered as a generalized coordinate.

![LiveScriptSnippet](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/Live_Script_Snippet4.PNG)

Comparing the analytical torque values and the code computed torque values:

![Result](https://github.com/average-engineer/Moving_Base_Robotic_Manipulator_Dynamic_Model/blob/main/Pole_Joint_Torque.png)

As can be seen, both the torques are more or less same across the time span. Thus, this dynamic model works.


