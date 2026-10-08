"""
SCARA kinematics -- YOUR implementation.

Fill in the four functions below from the math. Run the tests with:

    pytest test_kinematics.py -v

Work top to bottom; each function is used by the tests in order. Do not look
at any reference implementation until every test passes.

------------------------------------------------------------------------------
THE FRAME  (top-down view, this is the contract every function obeys)
------------------------------------------------------------------------------
    origin A : shoulder axis, at (0, 0)
    +x       : robot's right
    +y       : up on the plot; the mounting bracket is at +y
    patient  : at -y, so the arm reaches "down" into quadrants 3 and 4

    theta1 : shoulder angle, measured at A from the -y axis (straight at the
             patient), POSITIVE COUNTERCLOCKWISE.  theta1 = 0 -> link 1 points
             straight at the patient (toward -y).
    theta2 : elbow angle, measured at B from the extension of link 1, same
             positive sense.  theta2 = 0 -> arm fully extended (straight).

    Angles compound down the chain: link 2's absolute direction from the -y
    reference is (theta1 + theta2).

------------------------------------------------------------------------------
GEOMETRY
------------------------------------------------------------------------------
    L1 = 105 mm  (shoulder A -> elbow B)
    L2 =  75 mm  (elbow B -> tip C)

------------------------------------------------------------------------------
FORWARD KINEMATICS  (derive these; they are the heart of the exercise)
------------------------------------------------------------------------------
    A unit vector at angle theta from the -y axis, positive CCW, is:
        ( sin(theta), -cos(theta) )

    Elbow B is L1 along theta1:
        Bx =  L1 * sin(theta1)
        By = -L1 * cos(theta1)

    Tip C is B plus L2 along (theta1 + theta2):
        Cx = Bx + L2 * sin(theta1 + theta2)
        Cy = By - L2 * cos(theta1 + theta2)

------------------------------------------------------------------------------
JOINT LIMITS  (measured on the real arm; inverse() must respect these)
------------------------------------------------------------------------------
    shoulder theta1 in [-113, +67] degrees
    elbow    theta2 in [   0, 180] degrees
"""

import math

L1 = 105.0
L2 = 75.0

JOINT_LIMITS = {
    "shoulder": (-113.0, 67.0),
    "elbow": (0.0, 180.0),
}


def forward(theta1, theta2):
    """Forward kinematics.

    Args:
        theta1, theta2: joint angles in DEGREES, in the frame above.

    Returns:
        ((Bx, By), (Cx, Cy)) -- elbow and tip positions in mm.

    Hints:
        - math.sin/cos take RADIANS. Convert.
        - Return the elbow too; the tests check it and you'll want it later.
    """
    Bx = L1*math.sin(math.radians(theta1))
    By = -1*L1*math.cos(math.radians(theta1))
    Cx = Bx + L2*math.sin(math.radians(theta1 + theta2))
    Cy = By - L2*math.cos(math.radians(theta1 + theta2))
    return (Bx, By), (Cx, Cy)


def inverse(x, y, elbow="auto"):
    """Inverse kinematics: tip target -> joint angles.

    Args:
        x, y : desired tip position in mm.
        elbow: "pos", "neg", or "auto".
               A 2-link arm has TWO solutions for most points (elbow bends one
               way or the other). "pos" forces theta2 >= 0, "neg" forces
               theta2 <= 0, "auto" returns whichever lands inside the joint
               limits (prefer "pos" if both do).

    Returns:
        (theta1, theta2) in DEGREES.

    Raises:
        ValueError if the point is unreachable (outside the annulus) or, for
        "auto", if no solution lies within the joint limits.

    Derivation path:
        1. r = hypot(x, y). Reject if r > L1+L2 or r < |L1-L2|.
        2. Law of cosines gives theta2:
               cos(theta2) = (r^2 - L1^2 - L2^2) / (2*L1*L2)
           Clamp the argument to [-1, 1] before acos (float error).
        3. The bearing of the target from -y toward +x is:
               phi = atan2(x, -y)
        4. The tip sits at an angle delta off the link-1 direction:
               delta = atan2(L2*sin(theta2), L1 + L2*cos(theta2))
           so theta1 = phi - delta.
        5. The two elbow solutions use theta2 = +acos(...) and -acos(...),
           each with its own delta.
    """
    ## Reachability guard
    r = math.hypot(x,y)
    # if r < L1-L2 or r > L1+L2, automatically not reachable
    if r < L1 - L2 or r > L1 + L2:
        raise ValueError(f"({x:.0f}, {y:.0f}) geometrically unreachable")
    
    ## Because Cosine is an even function, we need to account for + and - acos [cos(-x) = cos(x)]
    # Creates 2 solutions/branches for each reachale (x,y)
    c2 = (x**2+y**2-L1**2-L2**2)/(2*L1*L2)
    c2 = max(-1.0, min(1.0, c2)) # clamp for float safety in case arithmetic is slightly off
    theta2 = math.acos(c2)

    # phi independent of theta2
    phi = math.atan2(x,-y)

    # function to generate theta1 and theta2 regardless of sign
    def solution(sign):
        t2 = sign * theta2
        delta = math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
        t1 = phi - delta
        return math.degrees(t1), math.degrees(t2)
    # generate solutions for branches we care about
    if elbow == "pos":
        return solution(+1)
    elif elbow == "neg":
        return solution(-1)
    else:
        for sign in (+1,-1):
            t1, t2 = solution(sign)
            if in_limits("shoulder", t1) and in_limits("elbow", t2):
                return t1, t2
    
    raise ValueError(f"({x:.0f}, {y:.0f}) unreachable. Angles out of Limits.")



def in_limits(joint, theta):
    """Return True if theta (degrees) is within that joint's limits."""
    if (JOINT_LIMITS[joint][0]) <= theta <= (JOINT_LIMITS[joint][1]):
        return True
    else:
        return False



def wrist_for(theta1, theta2, heading=0.0):
    """Wrist angle theta3 that holds the camera at a fixed absolute heading.

    The camera's absolute heading is theta1 + theta2 + theta3. To hold it at
    `heading`, solve for theta3.
    """
    return heading - theta1 - theta2
    raise NotImplementedError("write wrist_for()")
