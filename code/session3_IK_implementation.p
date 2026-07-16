import pybullet as p
import pybullet_data
import math

# ─────────────────────────────────────────────────────────────
# YOUR OWN IK FUNCTION
# Derived from first principles — see physics/IK_derivation.md
#
# Geometric IK for a 2-joint planar arm using:
# - Law of cosines to find θ₂
# - arctan + quadrant correction to find γ
# - Law of cosines to find α
# - θ₁ = γ − α (elbow-down) or γ + α (elbow-up)
#
# This closed-form solution only works for 2-joint planar arms.
# The 7-joint KUKA requires numerical Jacobian-based IK instead.
# ─────────────────────────────────────────────────────────────

def my_IK(x, y, L1, L2, elbow_up=False):
    """
    Inverse kinematics for a 2-joint planar arm.

    Inputs:
        x, y     : target end-effector position (metres)
        L1, L2   : link lengths (metres)
        elbow_up : False = elbow-down, True = elbow-up

    Returns:
        (theta1, theta2) in radians, or None if unreachable
    """

    # ── Step 0: Reachability check ───────────────────────────
    r = math.sqrt(x**2 + y**2)

    if r > L1 + L2:
        print(f"  Target unreachable: r={r:.3f} > L1+L2={L1+L2:.3f}")
        return None
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

    # Manual quadrant correction — see derivation paper
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


# ─────────────────────────────────────────────────────────────
# SANITY CHECKS — same as your paper
# ─────────────────────────────────────────────────────────────

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


# ─────────────────────────────────────────────────────────────
# VERIFY AGAINST FK
# The gold standard test: IK(FK(θ₁,θ₂)) should return θ₁,θ₂
# i.e. if we feed IK the position that FK produces,
# we should get back the original joint angles
# ─────────────────────────────────────────────────────────────

def verify_IK_with_FK():
    print("\n── IK verified against FK ───────────────")

    L1, L2 = 1.0, 1.0

    # Import your FK function
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

        # Inverse: position → angles
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


# ─────────────────────────────────────────────────────────────
# RUN EVERYTHING
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    run_sanity_checks()
    verify_IK_with_FK()
