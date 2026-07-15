# Inverse Kinematics  Derivation
## Hand-written proof
![IK derivation](IK_derivation_handwritten.pdf)
## Summary

**Goal:** given target position (x, y), find joint angles θ₁ and θ₂
**Key difference from fowrd kinematics:** Forward kinematics has a unique solution (a pair of angles gives one position). Inverse kinematics has two solutions (elbow-up and elbow-down) or no solution if the target is unreachable.
## key equations

$$\theta_2 = \pm \cos^{-1}\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\right)$$

$$\alpha = \cos^{-1}\left(\frac{L_1^2 + x^2 + y^2 - L_2^2}{2L_1\sqrt{x^2+y^2}}\right)$$

$$\theta_1 = \tan^{-1}\left(\frac{y}{x}\right) - \text{sgn}(\theta_2)\cdot\alpha 
\quad \text{[with quadrant correction]}$$
## Why this cannot be generalised to 7 joints
This geometric approach is based on the trigonometry of a 2-joint arm forming triangles with known side lengths.However, with 7 joints in 3D space, the system is redundant (infinitely many configurations reach the same target), thus the basic geometric approach cannot be used. Numerical methods using Jacobian matrices should be used instead (an approach used by PyBullet).

## Project context
Both elbow-up and elbow-down solutions are geometrically correct. The 
reinforcement learning agent will discover which configuration costs 
less energy for a given target.

[FK derivation →](FK_derivation.md)  
[Back to journal →](../journal/week2.md)
