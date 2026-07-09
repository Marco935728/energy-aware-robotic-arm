# Session 0 — [March 30 2026]

## What I did
Undestood how to trasnfer my coding enviroment form github to the macbook terminal, and impllentetna a new repository withmy journals.Studied the function that i will use from my code in order to maek the robotic hand

## Physics I learned
no physics learned 

## What confused me or didn't work
It was quite challenging and frustrating to move coding enviroment and link my test_physics.py code on both the macbook`s termina and GitHub

## Evidence
no evidence

# Session 1 — [April 6 2026]

## What I did
Set up a coding environment, implemented a robotic hand with 7 joints that can be moved manually with sliders and a target to be reached. The code also report the torque utilized and the distance from the robotic hand and the traget position.

## Physics I learned
week 1 physics was mostly rotational mechanics, which I had already learned prior to starting this personal project  

## What confused me or didn't work
I tried to move the joint to reach the target, but I was able to do so only after many failures.I understood that I need to automate the movement of the robotic hand if I want to eliminate human error 

## Evidence
(https://youtu.be/khyIA4fsZAQ) 


# Session 2 — [April 13 2026]

## What I did
I automated the movement of the hand to remove human failure using the calculateInverseKinematics function in pybullet( I have not studied inverse kinematics yet , i will use this function for now, but I will independently study it  soon.I used my code to make the torque reach a target at different position in order to analyze what is the relationship between energy and distance 
## Physics I learned
how to calculate the total energy used by the arm at different times using Riemann sum.
[derivation](../phisics/energy_integral.md)

## What confused me or didn't work
I was expecting a more clear inverse relationship between distance and energy, but it appears that distance is not the main determinant for use of energy.
## Evidence and  Evaluation of results 
[View this week's simulation code](../code/week1_controlled_experiment.py)

[table1]
| target_x | target_y | target_z | distance_m | energy_j |
| :--- | :--- | :--- | :--- | :--- |
| 0.3 | 0 | 0.6 | 0.0995 | 410.82 |
| 0.4 | 0.2 | 0.5 | 0.0735 | 499.34 |
| 0.5 | 0 | 0.4 | 0.0621 | 560.55 |
| 0.3 | 0.3 | 0.7 | 0.1125 | 333.62 |
| 0.6 | 0.1 | 0.3 | 0.0547 | 715.85 |


Firstly I tested  various positions and different (x,y,z) coordinates, and what I found was correlation between energy and accuracy(distance_m)[table1]. This interested me.Consequently,I decided to run  a better experiment where I isolated one axis and increased its distance systematically.
[table2]
### Baseline Experiment: Variable Isolation

#### 1. Varying X-Axis (Horizontal Reach)
| Test | Target [x, y, z] | Distance (m) | Energy (J) |
| :--- | :--- | :--- | :--- |
| vary_x_1 | [0.3, 0.0, 0.5] | 0.099 | 440.8 |
| vary_x_2 | [0.4, 0.0, 0.5] | 0.075 | 478.0 |
| vary_x_3 | [0.5, 0.0, 0.5] | 0.074 | 472.3 |
| vary_x_4 | [0.6, 0.0, 0.5] | 0.075 | 516.0 |
| vary_x_5 | [0.7, 0.0, 0.5] | 0.069 | 539.8 |

#### 2. Varying Y-Axis (lateral dispalcement)
| Test | Target [x, y, z] | Distance (m) | Energy (J) |
| :--- | :--- | :--- | :--- |
| vary_y_1 | [0.4, -0.2, 0.5] | 0.074 | 473.7 |
| vary_y_2 | [0.4, -0.1, 0.5] | 0.074 | 486.1 |
| vary_y_3 | [0.4, 0.0, 0.5] | 0.075 | 481.9 |
| vary_y_4 | [0.4, 0.1, 0.5] | 0.074 | 472.9 |
| vary_y_5 | [0.4, 0.2, 0.5] | 0.074 | 480.2 |

#### 3. Varying Z-Axis (Height)
| Test | Target [x, y, z] | Distance (m) | Energy (J) |
| :--- | :--- | :--- | :--- |
| vary_z_1 | [0.4, 0.0, 0.3] | 0.053 | 691.4 |
| vary_z_2 | [0.4, 0.0, 0.4] | 0.063 | 576.1 |
| vary_z_3 | [0.4, 0.0, 0.5] | 0.075 | 478.7 |
| vary_z_4 | [0.4, 0.0, 0.6] | 0.091 | 381.4 |
| vary_z_5 | [0.4, 0.0, 0.7] | 0.116 | 294.9 |

It is possible to conclude that height is the main determinant in energy consumption, and it has an inverse relationship: a lower height leads to more energy consumed.There is also a correlation between Horizontal reach(x-axis) and energy consumption, targets at greater horizontal distance consume more energy than closer ones – although the effect is moderate compared to height.Lateral displacement seems to have a negligible effect, and it does not show any clear pattern.The reason for this could be that  at lower height the different joints are exposed to more gravitational energy that needs more torque to be balanced, this is because when trying to grab something at low height the joint align non-vertically,on the contrary when a target is at high height, the joining “stack up”,supporting one another against the gravitational force,decreasing energy consuming.Similarity, trying to grab a target further away from the hand will mean the joint will need to “spread out” more , consuming more energy.It is also significant to look at the coordinate [0.4, 0.0, 0.5], which  appears in all three data set.The point produced consistent energy reading, 478J,482J and 479J respectively( a less than 1% variation), confirming the  reliability of the simulation, and its reproducibility between trials. 


# Session 3 — [9 July 2026]
## FK Implementation

### What I did
I derived Foward Kinematics equationfrom first principles, then I verified the equation trough "sanity check" and comapred my results against PyBullet.
### "Sanity check" results
── Sanity checks ────────────────────────
  ✓ PASSED | Fully extended
           got (2.0000, 0.0000), expected (2.0000, 0.0000)
  ✓ PASSED | L-shape
           got (1.0000, 1.0000), expected (1.0000, 1.0000)
  ✓ PASSED | Fully folded
           got (0.0000, 0.0000), expected (0.0000, 0.0000)
  ✓ PASSED | Folded unequal lengths
           got (0.5000, 0.0000), expected (0.5000, 0.0000)

### PyBullet comparison results
─ PyBullet comparison ──────────────────
Config            My FK (x,y)       PyBullet (x,z)    Error
  θ1=0°  θ2=0°     (0.417, 0.000)    (0.000, 0.694)    0.8101m
  θ1=45° θ2=0°     (0.295, 0.295)    (-0.021, 0.695)   0.5095m
  θ1=45° θ2=45°    (0.100, 0.375)    (0.146, 0.597)    0.2258m
  θ1=30° θ2=-45°   (0.389, -0.000)   (-0.220, 0.597)   0.8522m
  θ1=60° θ2=30°    (0.071, 0.398)    (0.058, 0.650)    0.2521m


### What the error tells me
As expected an error occurs when we try to apply Forward Kinematics planar trigonometry to our 3D KUKA arm. This happens because  the 3D arm operates also around the z-axis, an axis that does not exist in the 2D model. Therefore in order to accurately model the 3D arm, we would need to implement three-dimensional rotation matrices. Moreover, the KUKA arm contains  offsets and bends between links and joints that  cannot be perfectly modeled as simple straight lines(like our model was assuming).

## Evidence
[link to code/verify_fk.py]
[link to FK_derivation/FK_derivation.md]
