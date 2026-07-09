import pybullet as p
import pybullet_data
import math

def my_FK(theta1, theta2, l1, l2):
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return x, y


def run_sanity_checks(l1=1.0, l2=1.0):
    print("── Sanity checks ────────────────────────")
    
    checks = [
        ("Fully extended",        0,          0,      l1+l2, 0),
        ("L-shape",               0, math.pi/2,         l1, l2),
        ("Fully folded",          0,   math.pi,      l1-l2, 0),
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

    # Additional check — unequal link lengths
    x, y = my_FK(0, math.pi, 1.5, 1.0)
    expected_x, expected_y = 0.5, 0.0
    passed = abs(x - expected_x) < 0.001 and abs(y - expected_y) < 0.001
    print(f"  {'✓ PASSED' if passed else '✗ FAILED'} | "
          f"Folded unequal lengths")
    print(f"           got ({x:.4f}, {y:.4f}), "
          f"expected ({expected_x:.4f}, {expected_y:.4f})")

    print()
    if all_passed:
        print("All sanity checks passed — FK equations verified.")
    else:
        print("A check failed — review your FK equations.")
    return all_passed


def compare_with_pybullet():
    print("\n── PyBullet comparison ──────────────────")
    
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf",
                           [0, 0, 0], useFixedBase=True)

    # Link lengths read from zero-config URDF positions:
    # Link 0 is at z=0.277, Link 1 at z=0.419, Link 2 at z=0.694
    # L1 = distance from link 0 to link 1 = 0.419 - 0.277 = 0.142m
    # L2 = distance from link 1 to link 2 = 0.694 - 0.419 = 0.275m
    # NOTE: these are vertical distances at zero config — approximate
    L1 = 0.142
    L2 = 0.275

    test_configs = [
        (0.0,       0.0),
        (math.pi/4, 0.0),
        (math.pi/4, math.pi/4),
        (math.pi/6, -math.pi/4),
        (math.pi/3, math.pi/6),
    ]

    print(f"  {'Config':<30} {'My FK (x,y)':<24}"
          f"{'PyBullet (x,z)':<24} {'Error'}")
    print(f"  {'-'*95}")

    for theta1, theta2 in test_configs:
        p.resetJointState(robot_id, 0, theta1)
        p.resetJointState(robot_id, 1, theta2)
        p.stepSimulation()

        # Read link 2 — tip of our simplified 2-joint subsystem
        state = p.getLinkState(robot_id, 2)
        pos = state[0]

        # KUKA stands vertical so we compare our (x,y) planar model
        # against PyBullet's (x,z) — the arm moves in the x-z plane
        pb_x = pos[0]
        pb_z = pos[2]

        my_x, my_y = my_FK(theta1, theta2, L1, L2)

        # Error between our planar model and PyBullet's x-z plane
        error = math.sqrt((my_x - pb_x)**2 + (my_y - pb_z)**2)

        config_str = (f"θ1={math.degrees(theta1):.0f}°"
                      f" θ2={math.degrees(theta2):.0f}°")
        print(f"  {config_str:<30}"
              f"({my_x:.3f}, {my_y:.3f}){'':<12}"
              f"({pb_x:.3f}, {pb_z:.3f}){'':<12}"
              f"{error:.4f}m")

    p.disconnect()
    print()
    print("Note: residual errors reflect the 2D planar approximation")
    print("applied to a 3D arm. Eliminating this requires rotation matrices.")


if __name__ == "__main__":
    run_sanity_checks(l1=1.0, l2=1.0)
    compare_with_pybullet()
