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
## Matrix Multiplication and Reference Frames
In an arm of multiple joints that operates in a 3D space, each joint has a local reference frame. In order to find the end-effector position in the world frame, rotation matrices are chained together through matrix multiplication (each multiplication transforms the coordinates from one joint's frame to the next). For the 7-joint KUKA iiwa, this produces seven 4x4 transformation matrices (one per joint), which are multiplied together to obtain the forward kinematics solution.

This is why the 2D planar derivation cannot extend for the 7-joint arm. The scalar angles become matrices and the trigonometry used is substituted with a chain of matrix products.



[Back to journal →](../journal/week1.md)

