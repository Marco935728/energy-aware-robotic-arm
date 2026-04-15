
import pybullet as p
import pybullet_data
import time
import math
import csv
from datetime import datetime

# Setup 
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

num_joints   = p.getNumJoints(robot_id)
END_EFFECTOR = 6
DT           = 1.0 / 240.0
TORQUE_LIMITS = [320, 320, 176, 176, 110, 40, 40]

# Controlled target sets
# Each set varies ONE axis in equal 0.1m steps
# All other axes are held constant at a neutral position
# This lets us isolate which axis drives energy consumption

NEUTRAL_X = 0.4
NEUTRAL_Y = 0.0
NEUTRAL_Z = 0.5

target_sets = {
    "vary_x": [
        [0.3, NEUTRAL_Y, NEUTRAL_Z],
        [0.4, NEUTRAL_Y, NEUTRAL_Z],
        [0.5, NEUTRAL_Y, NEUTRAL_Z],
        [0.6, NEUTRAL_Y, NEUTRAL_Z],
        [0.7, NEUTRAL_Y, NEUTRAL_Z],
    ],
    "vary_y": [
        [NEUTRAL_X, -0.2, NEUTRAL_Z],
        [NEUTRAL_X, -0.1, NEUTRAL_Z],
        [NEUTRAL_X,  0.0, NEUTRAL_Z],
        [NEUTRAL_X,  0.1, NEUTRAL_Z],
        [NEUTRAL_X,  0.2, NEUTRAL_Z],
    ],
    "vary_z": [
        [NEUTRAL_X, NEUTRAL_Y, 0.3],
        [NEUTRAL_X, NEUTRAL_Y, 0.4],
        [NEUTRAL_X, NEUTRAL_Y, 0.5],
        [NEUTRAL_X, NEUTRAL_Y, 0.6],
        [NEUTRAL_X, NEUTRAL_Y, 0.7],
    ],
}

# Run experiment
results = []

for axis_name, targets in target_sets.items():
    print(f"\n── Testing {axis_name} ──────────────────")

    for target in targets:

        # Visual marker
        v_id = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 0, 0, 1]
        )
        p.createMultiBody(baseVisualShapeIndex=v_id, basePosition=target)

        # IK — black box for now, own derivation coming week 3
        joint_angles = p.calculateInverseKinematics(
            robot_id, END_EFFECTOR, target
        )

        # Energy integral: W = Σ |τᵢ · θ̇ᵢ| · dt
        total_energy = 0.0

        for step in range(600):
            p.stepSimulation()

            for i in range(num_joints):
                p.setJointMotorControl2(
                    robot_id, i,
                    p.POSITION_CONTROL,
                    targetPosition=joint_angles[i],
                    force=TORQUE_LIMITS[i]
                )
                state         = p.getJointState(robot_id, i)
                tau           = state[3]
                theta_dot     = state[1]
                total_energy += abs(tau * theta_dot) * DT

            time.sleep(DT)

        # Final end-effector position
        hand_pos = p.getLinkState(robot_id, END_EFFECTOR)[0]
        dist = math.sqrt(sum(
            (hand_pos[i] - target[i])**2 for i in range(3)
        ))

        print(f"  {axis_name} | target={target} | "
              f"dist={dist:.3f}m | energy={total_energy:.1f}J")

        results.append({
            "axis":     axis_name,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "distance": round(dist, 4),
            "energy":   round(total_energy, 2),
        })

        # Reset arm before next target
        for i in range(num_joints):
            p.resetJointState(robot_id, i, 0)
        time.sleep(0.3)

p.disconnect()

# Save CSV 
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename  = f"results/controlled_experiment_{timestamp}.csv"

with open(filename, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "axis", "target_x", "target_y",
        "target_z", "distance", "energy"
    ])
    writer.writeheader()
    writer.writerows(results)

print(f"\nData saved to {filename}")
