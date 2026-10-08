"""
Tests for your SCARA kinematics. Run:  pytest test_kinematics.py -v

Read each test before you write the code it checks -- the test IS the spec.
Work through the classes top to bottom.
"""

import math
import random

import pytest

from kinematics import L1, L2, forward, inverse, in_limits, wrist_for

TOL = 0.01          # mm / degree tolerance for float comparisons


# ===========================================================================
# 1. FORWARD KINEMATICS -- start here
# ===========================================================================
class TestForward:

    def test_zero_pose_points_at_patient(self):
        # theta1=0, theta2=0: arm fully extended straight at the patient (-y).
        # Tip should be at (0, -(L1+L2)) = (0, -180).
        (_, _), (cx, cy) = forward(0, 0)
        assert cx == pytest.approx(0.0, abs=TOL)
        assert cy == pytest.approx(-180.0, abs=TOL)

    def test_elbow_position_independent_of_theta2(self):
        # The elbow B depends only on theta1. Two different theta2 values must
        # give the same B.
        (b1x, b1y), _ = forward(30, 40)
        (b2x, b2y), _ = forward(30, 120)
        assert b1x == pytest.approx(b2x, abs=TOL)
        assert b1y == pytest.approx(b2y, abs=TOL)

    def test_shoulder_ninety_points_right(self):
        # theta1=+90 (CCW from -y) points link 1 toward +x (robot's right).
        # With theta2=0, tip at (L1+L2, 0) = (180, 0).
        _, (cx, cy) = forward(90, 0)
        assert cx == pytest.approx(180.0, abs=TOL)
        assert cy == pytest.approx(0.0, abs=TOL)

    def test_shoulder_negative_ninety_points_left(self):
        _, (cx, cy) = forward(-90, 0)
        assert cx == pytest.approx(-180.0, abs=TOL)
        assert cy == pytest.approx(0.0, abs=TOL)

    def test_home_pose(self):
        # theta1=0, theta2=90. Elbow straight down at (0,-105); forearm then
        # swings to +x. Tip expected at (75, -105).
        (bx, by), (cx, cy) = forward(0, 90)
        assert (bx, by) == pytest.approx((0.0, -105.0), abs=TOL)
        assert (cx, cy) == pytest.approx((75.0, -105.0), abs=TOL)

    def test_elbow_is_L1_from_origin_always(self):
        # |A->B| must equal L1 for any theta1.
        for t1 in (-113, -40, 0, 25, 67):
            (bx, by), _ = forward(t1, 55)
            assert math.hypot(bx, by) == pytest.approx(L1, abs=TOL)

    def test_reach_matches_law_of_cosines(self):
        # |A->C| should equal sqrt(L1^2 + L2^2 + 2 L1 L2 cos(theta2)).
        for t2 in (0, 30, 90, 150, 180):
            _, (cx, cy) = forward(10, t2)
            r = math.hypot(cx, cy)
            expect = math.sqrt(L1**2 + L2**2 + 2*L1*L2*math.cos(math.radians(t2)))
            assert r == pytest.approx(expect, abs=TOL)


# ===========================================================================
# 2. INVERSE KINEMATICS
# ===========================================================================
class TestInverse:

    def test_round_trip_random(self):
        # forward then inverse must return to the same joint angles, across
        # the whole legal joint space.
        random.seed(1)
        for _ in range(500):
            t1 = random.uniform(-113, 67)
            t2 = random.uniform(1, 179)          # avoid exact singularities
            _, (cx, cy) = forward(t1, t2)
            s1, s2 = inverse(cx, cy, elbow="pos")
            _, (rx, ry) = forward(s1, s2)
            assert (rx, ry) == pytest.approx((cx, cy), abs=TOL)

    def test_both_elbow_branches_reach_same_point(self):
        _, target = forward(20, 70)
        for branch in ("pos", "neg"):
            s1, s2 = inverse(*target, elbow=branch)
            _, reached = forward(s1, s2)
            assert reached == pytest.approx(target, abs=TOL)

    def test_pos_branch_has_nonneg_elbow(self):
        _, target = forward(15, 95)
        _, t2 = inverse(*target, elbow="pos")
        assert t2 >= -TOL

    def test_neg_branch_has_nonpos_elbow(self):
        _, target = forward(15, 95)
        _, t2 = inverse(*target, elbow="neg")
        assert t2 <= TOL

    def test_unreachable_too_far_raises(self):
        with pytest.raises(ValueError):
            inverse(0, -200)         # beyond L1+L2 = 180

    def test_unreachable_too_close_raises(self):
        with pytest.raises(ValueError):
            inverse(5, 0)            # inside |L1-L2| = 30

    def test_auto_prefers_in_limit_solution(self):
        # A point whose "pos" solution violates the shoulder limit but whose
        # "neg" solution is legal should return the legal one under "auto".
        # Point roughly to the far robot-left, near the limit edge.
        _, target = forward(-100, 60)
        s1, s2 = inverse(*target, elbow="auto")
        assert in_limits("shoulder", s1)
        assert in_limits("elbow", s2)


# ===========================================================================
# 3. LIMITS
# ===========================================================================
class TestLimits:

    def test_within(self):
        assert in_limits("shoulder", 0)
        assert in_limits("elbow", 90)

    def test_on_boundary(self):
        assert in_limits("shoulder", -113)
        assert in_limits("shoulder", 67)
        assert in_limits("elbow", 0)
        assert in_limits("elbow", 180)

    def test_outside(self):
        assert not in_limits("shoulder", -114)
        assert not in_limits("shoulder", 68)
        assert not in_limits("elbow", -1)
        assert not in_limits("elbow", 181)


# ===========================================================================
# 4. WRIST (camera orientation hold)
# ===========================================================================
class TestWrist:

    def test_heading_is_held(self):
        # By definition theta1+theta2+theta3 == heading.
        for t1, t2, h in [(0, 90, 0), (-40, 120, 0), (30, 60, 15)]:
            t3 = wrist_for(t1, t2, heading=h)
            assert (t1 + t2 + t3) == pytest.approx(h, abs=TOL)

    def test_zero_pose_zero_wrist(self):
        # At theta1=theta2=0 with heading 0, the wrist should also be 0.
        assert wrist_for(0, 0, heading=0) == pytest.approx(0.0, abs=TOL)
