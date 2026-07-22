# Reinforcement Learning 

In Reinforcement Learning (RL) an "agent" learns how to make decisions by interacting with the environment. 
The agent observes the current state then it takes an action and receives a reward signal at each time step. 
Over time the agent learns a policy that should supposedly maximize the cumulative reward.

The agent  discovers the correct action through trial-and-error and reward feedback.

##  Need for RL
Since the solution of a path for a 7-joint arm is redundant, the energy-optimal path cannot be conclusively found in an analytical way 
(there are too many possible paths to explore). Thus, RL will be used to autonomously find an efficient solution 
by exploring the environment without requiring programming for every case.

## components 

**State:** It's what the agent observes at each timestep. 
For this project it's the current joint angles, joint velocities, end-effector position, target position, and distance to target (21 values total)

**Action:** It's what the agent can control. For this project it is the joint angles it moves (7 values, one per joint)
**Reward:** t's the signal after the action of the arm that will help the agent to evaluate the most effective policy.

$$r_t = -w_1 \cdot d_t - w_2 \cdot P_t + R_{success}$$
Where $d_t$ is the distance to the target, $P_t = \sum_i |\tau_i \cdot \dot{\theta}_i|$ is the instantaneous mechanical power (where energy is given by the integral $W = \int \tau \dot{\theta} \, dt$), and $R_{\text{success}}$ is an award obtained when the target is reached within $0.05\text{ m}$. The weights $w_1$ and $w_2$ control the balance between accuracy and efficiency.

## Why PPO
While various RL algorithms exist, this project chose Proximal Policy Optimization (PPO), due to the following reasons:

**Stability:** In order to not make the agent change its behavior too drastically, PPO limits the policy update.
Due to this limit, training becomes more stable and more protected from harmfully large policy updates.

**Continuous action spaces:** The arm's joint angles are continuous values and PPO works on continuous action space naturally.

**Trustability :** PPO is used also in industry, and it is a well-documented and standardized algorithm.


## The research question

Often standard robotic controls prioritize speed (reaching the target as fast as possible). However, this project focuses on a different goal:
making an agent learn to reach a target using minimum energy and analyzing whether the optimal strategy depends on target geometry.

[In my controlled experiment I have discovered that height (z-axis) is the main reason for energy consumption. 
It will be interesting to observe whether the RL agent confirms this finding 
or reveals unexpected efficient paths that were not considered/noticed in the controlled experiment.]


[Back to journal →](../journal/week1.md)
[Energy integral derivation →](FK_derivation.md)
