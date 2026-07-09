# Forward Kinematics Derivation

## Hand-written proof
![FK derivation](FK_derivation_handwritten.pdf)

## Summary

**Goal:** given joint angles θ₁, θ₂, find the end-effector position (x, y)

**Key idea:** θ₂ is measured relative to link 1, so the absolute 
angle of link 2 is (θ₁ + θ₂). Each link is added 
as a vector projection.

**Result**

$$x = l_1\cos(\theta_1) + l_2\cos(\theta_1 + \theta_2)$$
$$y = l_1\sin(\theta_1) + l_2\sin(\theta_1 + \theta_2)$$

**General form for n joints:**

$$x = \sum_{k=1}^{n} l_k \cos\left(\sum_{i=1}^{k} \theta_i\right)$$

**Note:** The KUKA iiwa works in 3D, thus it requires rotation matrices and not scalar trigonometry.


[Back to journal →](../journal/week2.md)

