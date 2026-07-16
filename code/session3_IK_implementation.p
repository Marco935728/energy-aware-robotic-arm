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

    # Additional check  unequal link lengths
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

        state = p.getLinkState(robot_id, 2)
        pos = state[0]

        pb_x = pos[0]
        pb_z = pos[2]

        my_x, my_y = my_FK(theta1, theta2, L1, L2)


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
    compare_with_pybullet()        return None
    if r < abs(L1 - L2):
        print(f"  Target unreachable: r={r:.3f} < |L1-L2|={abs(L1-L2):.3f}")
        return None

    # ── Step 2: Solve for θ₂ ────────────────────────────────
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    # Clamp to [-1, 1] to handle floating point edge cases
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2 = math.acos(cos_theta2)
    if elbow_up:
        theta2 = -theta2  # elbow-up uses negative θ₂

    # ── Step 3a: Find γ with quadrant correction ─────────────
    gamma_raw = math.atan(y / x) if x != 0 else math.pi / 2

    # Manual quadrant correction 
    if x < 0:
        gamma = gamma_raw + math.pi   # Quadrants II and III
    else:
        gamma = gamma_raw             # Quadrants I and IV

    # ── Step 3b: Find α ──────────────────────────────────────
    cos_alpha = (L1**2 + x**2 + y**2 - L2**2) / (2 * L1 * r)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    # ── Step 4: Solve for θ₁ ────────────────────────────────
    if elbow_up:
        theta1 = gamma + alpha
    else:
        theta1 = gamma - alpha

    return theta1, theta2




def run_sanity_checks():
    print("── IK Sanity checks ─────────────────────")

    L1, L2 = 10.0, 10.0

    checks = [
        ("Full extension (x-axis)",  20,  0,   0,  0),
        ("Right angle (elbow-down)", 10, 10,   0, 90),
    ]

    all_passed = True
    for name, x, y, exp_t1, exp_t2 in checks:
        result = my_IK(x, y, L1, L2, elbow_up=False)
        if result is None:
            print(f"  ✗ FAILED | {name} — unreachable")
            all_passed = False
            continue

        t1, t2 = result
        t1_deg = math.degrees(t1)
        t2_deg = math.degrees(t2)

        passed = (abs(t1_deg - exp_t1) < 0.1 and
                  abs(t2_deg - exp_t2) < 0.1)
        status = "✓ PASSED" if passed else "✗ FAILED"
        if not passed:
            all_passed = False

        print(f"  {status} | {name}")
        print(f"           got θ₁={t1_deg:.2f}°, θ₂={t2_deg:.2f}°  "
              f"expected θ₁={exp_t1}°, θ₂={exp_t2}°")

    # Reachability check test
    print()
    print("  Testing reachability check:")
    result = my_IK(25, 0, L1, L2)
    if result is None:
        print("  ✓ PASSED | Unreachable target correctly rejected")
    else:
        print("  ✗ FAILED | Unreachable target should have been rejected")

    print()
    if all_passed:
        print("All IK sanity checks passed.")
    else:
        print("A check failed — review IK equations.")




def verify_IK_with_FK():
    print("\n── IK verified against FK ───────────────")

    L1, L2 = 1.0, 1.0

    # Import  FK function
    def FK(t1, t2):
        x = L1*math.cos(t1) + L2*math.cos(t1+t2)
        y = L1*math.sin(t1) + L2*math.sin(t1+t2)
        return x, y

    test_angles = [
        (0,           0),
        (math.pi/4,   math.pi/4),
        (math.pi/3,   math.pi/6),
        (math.pi/6,  -math.pi/4),
        (math.pi/2,   math.pi/3),
    ]

    print(f"  {'Original angles':<28} {'FK position':<24} "
          f"{'IK recovered':<28} {'Error'}")
    print(f"  {'-'*100}")

    all_passed = True
    for t1_orig, t2_orig in test_angles:
        # Forward: angles → position
        x, y = FK(t1_orig, t2_orig)

        # Inverse: position -> angles
        result = my_IK(x, y, L1, L2, elbow_up=False)
        if result is None:
            print(f"  IK returned None for ({x:.3f}, {y:.3f})")
            continue

        t1_rec, t2_rec = result

        # Error in degrees
        err1 = abs(math.degrees(t1_orig - t1_rec))
        err2 = abs(math.degrees(t2_orig - t2_rec))

        passed = err1 < 0.1 and err2 < 0.1
        if not passed:
            all_passed = False
        status = "✓" if passed else "✗"

        orig_str = (f"θ₁={math.degrees(t1_orig):.1f}° "
                    f"θ₂={math.degrees(t2_orig):.1f}°")
        pos_str  = f"({x:.3f}, {y:.3f})"
        rec_str  = (f"θ₁={math.degrees(t1_rec):.1f}° "
                    f"θ₂={math.degrees(t2_rec):.1f}°")

        print(f"  {status} {orig_str:<27} {pos_str:<24} "
              f"{rec_str:<27} {err1:.3f}°, {err2:.3f}°")

    print()
    if all_passed:
        print("IK perfectly inverts FK — derivation verified.")
    else:
        print("Some angles did not recover — check elbow configuration.")



if __name__ == "__main__":
    run_sanity_checks()
    verify_IK_with_FK()
