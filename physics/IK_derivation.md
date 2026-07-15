# Inverse Kinematics  Derivation
## Hand-written proof
![IK derivation](IK_derivation_handwritten.jpg)
## Summary

**Goal:** given target position (x, y), find joint angles θ₁ and θ₂
**Key difference from fowrd kinematics:** foward kinematics has a unique solution( a pair of 
angles gives one position). Inverse kinematics has two solutions (elbow-up and 
elbow-down) or  no solution if the target is unreachable.
## key equations

$$\theta_2 = \pm \cos^{-1}\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\right)$$

$$\alpha = \cos^{-1}\left(\frac{L_1^2 + x^2 + y^2 - L_2^2}{2L_1\sqrt{x^2+y^2}}\right)$$

$$\theta_1 = \tan^{-1}\left(\frac{y}{x}\right) - \text{sgn}(\theta_2)\cdot\alpha 
\quad \text{[with quadrant correction]}$$
