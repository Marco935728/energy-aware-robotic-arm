# The Jacobian Matrix
## Hand-written derivation
![Jacobian derivation](jacobian_derivation_handwritten.pdf)
## Motivation 
Foward kinemtics(FK) gives the end-effector position from joint angles. Howerver, for control and opitmization we also need to know how fast the end-effector moves as the joints rotate at velocity θ̇. This requires differentiating the FK equations with respect to joint angle, which produce the Jacobian matrix.
## Partial derivative
A partial derivative differentiates a function of multiple 
variables with respect to one variable, treating all others 
as constants. The four partial derivatives of the FK equations are derivered on the "Hand-written derivation".
## Result
$$J = \begin{bmatrix} -L_1\sin\theta_1 - L_2\sin(\theta_1+\theta_2) & -L_2\sin(\theta_1+\theta_2) \\ L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) & L_2\cos(\theta_1+\theta_2) \end{bmatrix}$$
## Meaning 
Each column reveals how the end-effector moves when only one join rotates:
- Column 1: end-effector velocity when only joint 1 spins
- Column 2: end-effector velocity when only joint 2 spins
The full velocity is their weighted sum:
$$\begin{bmatrix} v_x \\ v_y \end{bmatrix} = J \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \end{bmatrix}$$

## Why thsi does not work for a 7 joints
A 2-joint planar arm has a  Jacobian matrix of  2×2, which is  square and 
invertible. Joint velocities: θ̇ = J⁻¹ẋ.
However, a 7-joint KUKA iiwa has an end-effecto with 6 degrees of freedom(3 position and 3 orientation), thsu making the Jacobian matrix 6x7
(six rows and seven columns).Since thsi matric is non-square, there is not a stadard inverse. Consequently the Moore-Penrose pseudoinverse is used: 

$$J^+ = J^T(JJ^T)^{-1}$$

This finds the minimum joint velocity solution among all infinite solutions that exist due to redundancy. This is what 
PyBullet's "calculateInverseKinematics" uses.

## Matrix multiplication
{...}
## Connection to this project's energy integral
The Jacobian matrix directly connects to W = ∫τθ̇dt.Using  ẋ = Jθ̇ the Jacobian matrix can relate the joitn velocities θ̇ to end-effector velocity.Moreover, trough deeper manipulation (which is beyond the scope of thsi derivation)  joint torques and end-effector forces can be releated with the transpose Jacobian: τ = Jᵀf.



The Jacobian relates joint velocities θ̇ to end-effector velocity 
via ẋ = Jθ̇. A deeper result in manipulator dynamics — beyond the 
scope of this derivation — shows that joint torques and end-effector 
forces are related through the transpose Jacobian: τ = Jᵀf. 
Thus, the energy intergal W = ∫τθ̇dt is related to the arm configuration, the jacobian matric determinesd how efficiently 
joint effort becomes  useful end-effector motion.Therefore, the RL agent also optimizes effort relative to the arm`s instantaneous configuration.

 The RL 
agent therefore does not just minimize raw torque, but implicitly 
optimizes effort relative to the arm's instantaneous configuration.

[FK derivation →](FK_derivation.md)  
[IK derivation →](IK_derivation.md)  
[Back to journal →](../journal/week1.md)
