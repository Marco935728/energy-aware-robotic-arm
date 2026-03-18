import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

num_joints = p.getNumJoints(robot_id)
slider_ids = [p.addUserDebugParameter(f"Joint_{i}", -3, 3, 0) for i in range(num_joints)]

try:
    while True:
        p.stepSimulation()
        total_torque = 0
        for i in range(num_joints):
            target_pos = p.readUserDebugParameter(slider_ids[i])
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, target_pos)

            # NEW: Get the force/torque being used by this joint
            joint_state = p.getJointState(robot_id, i)
            total_torque += abs(joint_state[3]) # Index 3 is the applied torque

        if int(time.time() * 10) % 5 == 0: # Print every half second
            print(f"Total Effort (Torque): {total_torque:.2f} Nm")

        time.sleep(1./240.)
except KeyboardInterrupt:
    p.disconnect()
