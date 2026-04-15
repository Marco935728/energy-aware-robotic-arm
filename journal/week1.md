# Week 0 — [March 30 2026]

## What I did
Undestood how to trasnfer my coding enviroment form github to the macbook terminal, and impllentetna a new repository withmy journals.Studied the function that i will use from my code in order to maek the robotic hand

## Physics I learned
no physics learned 

## What confused me or didn't work
It was quite challenging and frustrating to move coding enviroment and link my test_physics.py code on both the macbook`s termina and GitHub

## Evidence
no evidence

# Week 1 — [April 6 2026]

## What I did
Set up a coding environment, implemented a robotic hand with 7 joints that can be moved manually with sliders and a target to be reached. The code also report the torque utilized and the distance from the robotic hand and the traget position.

## Physics I learned
week 1 physics was mostly rotational mechanics, which I had already learned prior to starting this personal project  

## What confused me or didn't work
I tried to move the joint to reach the target, but I was able to do so only after many failures.I understood that I need to automate the movement of the robotic hand if I want to eliminate human error 

## Evidence
(https://youtu.be/khyIA4fsZAQ) 


# Week 2 — [April 13 2026]

## What I did
I automated the movement of the hand to remove human failure using the calculateInverseKinematics function in pybullet( I have not studied inverse kinematics yet , i will use this function for now, but I will independently study it  soon.I used my code to make the torque reach a target at different position in order to analyze what is the relationship between energy and distance 
## Physics I learned
how to calculate the total energy used by the arm at different times using Riemann sum. 

## What confused me or didn't work
I was expecting a more clear inverse relationship between distance and energy, but it appears that distance is not the main determinant for use of energy.
## Evidence and  Evaluation of results 
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

