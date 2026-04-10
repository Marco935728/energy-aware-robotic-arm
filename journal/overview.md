# How this project works

## The core idea
A robotic arm wastes energy when it moves carelessly — 
high torque, jerky motion, inefficient paths. This project 
trains an AI to find paths that reach the target while 
minimizing wasted energy.

## The physics in plain language
Every joint in a robot arm acts like a motor. That motor 
produces torque (a rotational force, measured in Nm) and 
spins at some angular velocity (rad/s). 

Power at a joint = torque × angular velocity  
Energy = power integrated over time

So: **W = ∫τθ̇ dt**

This is not an approximation. This is the exact mechanical 
work done by each joint. My code computes this at every 
simulation timestep.

[Full mathematical derivation →](../physics/energy_integral.md)

## The methodology
1. Define a target position
2. Move the arm there using inverse kinematics
3. Measure total energy consumed
4. Train an RL agent to minimize that energy
5. Compare: naive control vs optimized control

[See the weekly journal for how each step developed →](week1.md)
