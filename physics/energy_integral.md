
Work: $W$
Torque: $\tau$
Velocity: $\dot{\theta}$
Time step: $\Delta t$
Sum: $\sum$
Integral: $\int$
# The Energy Integral: Full Derivation

## 1. Rotational Work from First Principles
In linear mechanics, Work is the product of Force and Displacement ($W = F \cdot d$). In the rotational mechanics, we translate these to **Torque** ($\tau$) and **Angular Displacement** ($\theta$).

$$W = \tau \cdot \theta$$

## 2. Dealing with Time-Varying Torque
Because the robot's torque changes every millisecond due to gravity and momentum, we must use the **Power-Time integral**.

Mechanical Power ($P$) is defined as:
$$P = \tau \cdot \omega$$
*(Where $\omega$ is angular velocity $\dot{\theta}$)*

Total Mechanical Work ($W$) is the integral of Power over time:
$$W = \int_{0}^{T} | \tau(t) \cdot \dot{\theta}(t) | \, dt$$

## 3. Discretization for Simulation
Since PyBullet runs in discrete timesteps ($\Delta t = \frac{1}{240}$s), my code implements a **Riemann Sum** to approximate the integral:

$$W \approx \sum_{i=1}^{n} | \tau_i \cdot \dot{\theta}_i | \cdot \Delta t$$

This allows me to track the "Energy Cost" of every movement in real-time.
