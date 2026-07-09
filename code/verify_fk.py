import pybullet as p
import pybullet_data
import math


def my_FK(theta1, theta2, l1, l2):
    """
    Forward kinematics for a 2-joint planar arm.
    
    Inputs:
        theta1 : angle of joint 1 from horizontal (radians)
        theta2 : angle of joint 2 relative to link 1 (radians)
        l1     : length of link 1 (meters)
        l2     : length of link 2 (meters)
    
    Returns:
        (x, y) : end-effector position in meters
    """
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return x, y



def run_sanity_checks(l1=1.0, l2=1.0):
    print("── Sanity checks ────────────────────────")
    
    checks = [
        ("Fully extended",   0,           0,          l1+l2, 0),
        ("L-shape",          0,  math.pi/2,               l1, l2),
        ("Fully folded",     0,   math.pi,            l1-l2, 0),
    ]
    
    all_passed = True
    for name, t1, t2, expected_x, expected_y in checks:
        x, y = my_FK(t1, t2, l1, l2)
        passed = (abs(x - expected_x) < 0.001 and 
                  abs(y - expected_y) < 0.001)
        status = "✓ PASSED" if passed else "✗ FAILED"
        if not passed:
            all_passed = False
        print(f"  {status} | {name}")
        print(f"           got ({x:.4f}, {y:.4f}), "
              f"expected ({expected_x:.4f}, {expected_y:.4f})")
    
    print()
    if all_passed:
        print("All sanity checks passed, FK equations verified.")
    else:
        print("A check failed, review your FK equations.")
    return all_passed



def compare_with_pybullet():
    print("\n── PyBullet comparison ──────────────────")
    
    p.connect(p.DIRECT)  # no GUI needed for this test
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", 
                           [0, 0, 0], useFixedBase=True)

    # KUKA link lengths (approximate, from URDF geometry)
    # Joint 0 to Joint 1 vertical offset ~ 0.34m
    # Joint 1 to Joint 2 ~ 0.40m
    # These are approximate — the KUKA is 3D so projection
    # onto a 2D plane introduces some error
    L1 = 0.34
    L2 = 0.40

    # Test configurations: (theta1, theta2) in radians
    test_configs = [
        (0.0,          0.0),
        (math.pi/4,    0.0),
        (math.pi/4,    math.pi/4),
        (math.pi/6,   -math.pi/4),
        (math.pi/3,    math.pi/6),
    ]

    print(f"  {'Config':<30} {'My FK (x,y)':<24} "
          f"{'PyBullet (x,y)':<24} {'Error'}")
    print(f"  {'-'*90}")

    for theta1, theta2 in test_configs:

        # Set joint angles in PyBullet
        p.resetJointState(robot_id, 0, theta1)
        p.resetJointState(robot_id, 1, theta2)
        p.stepSimulation()

        # Get end-effector position from PyBullet (joint 2 position)
        link_state = p.getLinkState(robot_id, 1)
        pb_pos = link_state[0]  # world position of link 1 tip
        pb_x, pb_y = pb_pos[0], pb_pos[1]


        my_x, my_y = my_FK(theta1, theta2, L1, L2)


        error = math.sqrt((my_x - pb_x)**2 + (my_y - pb_y)**2)

        config_str = f"θ1={math.degrees(theta1):.0f}° θ2={math.degrees(theta2):.0f}°"
        print(f"  {config_str:<30} "
              f"({my_x:.3f}, {my_y:.3f}){'':<12}"
              f"({pb_x:.3f}, {pb_y:.3f}){'':<12}"
              f"{error:.4f}m")

    p.disconnect()
    print()
    print("Note: errors > 0 are expected — the KUKA is a 3D arm")
    print("and our FK model is 2D planar. The errors reveal exactly")
    print("why rotation matrices are needed for the full 7-joint case.")




if __name__ == "__main__":
    run_sanity_checks(l1=1.0, l2=1.0)
    compare_with_pybullet()
