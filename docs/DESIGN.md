# Design Notes — Eyedropper SCARA Arm

A design journal for the arm: the reasoning, the derivations, the empirical
bring-up, and the open problems. For how to build and run the system, see the
[README](../README.md); this document is the "why" behind it.

---

## 1. Abstract

An affordable SCARA arm that tracks a target eye and delivers eyedrop
medication with precision, aimed at patients who lack the dexterity to apply
drops themselves. The design deliberately uses low-cost motors and drivers
(hobby servos, a PWM breakout) to keep the platform cheap and reproducible,
trading actuator precision for accessibility — a trade managed in software and
mechanism rather than by buying better hardware.

---

## 2. Kinematic frame

All kinematics are derived from a **top-down view** of the arm.

- Origin **A** at the shoulder (first motor's axis of rotation).
- **+x** to the robot's right, **+y** up on the plot.
- The mounting bracket is at **+y**; the patient sits at **−y**, so the arm
  works *downward* into **quadrants 3 and 4**.
- **θ₁** (shoulder) measured from the −y axis (straight at the patient),
  positive counterclockwise. θ₁ = 0 points link 1 at the patient.
- **θ₂** (elbow) measured from the extension of link 1, same positive sense.
  θ₂ = 0 is a fully extended arm.
- **θ₃** (wrist) measured from the extension of link 2; holds the camera at a
  fixed world orientation.

Angles compound down the chain, so the camera's absolute heading is θ₁+θ₂+θ₃.

**Link lengths:** l₁ = 105 mm (shoulder→elbow), l₂ = 75 mm (elbow→tip).

### Forward kinematics

```
Bx =  l₁·sin(θ₁)
By = −l₁·cos(θ₁)
Cx =  l₁·sin(θ₁) + l₂·sin(θ₁+θ₂)
Cy = −l₁·cos(θ₁) − l₂·cos(θ₁+θ₂)
```

Each link contributes `l·(sin, −cos)` evaluated at the cumulative angle up to
that link — the pattern that generalizes to any number of links and explains
why the camera heading is a simple sum.

### Inverse kinematics

Solved in closed form. The reach distance `r = √(x²+y²)` depends only on the
elbow angle (the shoulder term cancels when you square-and-add the FK
equations), giving θ₂ by the law of cosines:

```
cos(θ₂) = (r² − l₁² − l₂²) / (2·l₁·l₂)
```

θ₂ = ±acos(...) yields the two elbow branches ("elbow up/down"). The shoulder
angle is then the bearing to the target minus the offset the elbow bend
introduces:

```
φ = atan2(x, −y)                              # bearing from −y toward +x
δ = atan2(l₂·sin θ₂, l₁ + l₂·cos θ₂)          # offset of link 1 from the A→C line
θ₁ = φ − δ
```

The sign of θ₂ flows through δ automatically, so both branches come out of the
same formula. The IK rejects targets outside the annulus
(`|l₁−l₂| ≤ r ≤ l₁+l₂`) and, in "auto" mode, returns whichever branch lands
inside the joint limits. The whole kinematics library is covered by a 19-case
`pytest` suite (FK spot checks, randomized forward→inverse→forward round trips,
reachability guards).

---

## 3. Controls & hardware

- **Compute:** Raspberry Pi 4 (hardware I/O) + a workstation (control, analysis,
  and later policy training).
- **Driver:** PCA9685 — a 16-channel, 12-bit PWM controller over I²C, 4096 steps
  per channel. Offloads PWM generation from the Pi so servo timing is
  jitter-free.
- **Actuators:** 3× MG996R hobby servos.
- **Power:** servos on a separate 5–6 V supply (never the Pi's rail); a common
  ground tie between Pi, PCA9685, and supply; bulk capacitance on the servo rail.

Each motor's channel matches the angle it controls: θ₁→channel 1, θ₂→channel 2,
θ₃→channel 3.

---

## 4. Bring-up

A deliberately incremental, rung-by-rung bring-up, each step verifying the one
below it before adding risk.

- **`rung0_hello.py`** — nudges each channel a few degrees to verify wiring and
  confirm which physical joint each channel drives.
- **`rung1_limits.py`** — finds each servo's mechanical limits empirically:
  move to 90°, then step outward in 5° increments until the joint approaches a
  mechanical stop.

Empirical limits found (raw servo-command angles, **not yet** calibrated to the
kinematic origin):

| Motor | Lower limit | Upper limit |
|---|---|---|
| 1 (shoulder) | ~140, continuing past 0 | — |
| 2 (elbow) | 85 | past 180 |
| 3 (wrist) | 0 | 180 (unconstrained) |

**Observations logged during bring-up** (the problems, before the fixes):

- Large commanded steps produced fast, jerky motion — needed smoothing.
- Commanded 0° caused audible noise and shaking (servo hunting at the deadband
  extreme).
- The shoulder rest position starts near 140° and swings well past 0° into
  roughly −130°, so its usable range had to be mapped and then related back to
  the kinematic frame.

These raw findings are the precursor to the calibration in §5.

---

## 5. Calibration — kinematic frame vs. servo commands

The raw servo angles above are relative only to each servo's internal encoder.
The kinematics speak a different language (angles about the defined origin), so
a fixed affine map lives between them, in exactly one place (the config):

```
servo_cmd = direction · θ_kinematic + offset
```

Measured usable ranges and mounting handedness (motors 1 and 2 are mounted
mirror-image, hence `direction = −1`):

| Joint | Kinematic range | direction | offset | cmd at θ=0 |
|---|---|---|---|---|
| shoulder | θ₁ ∈ [−113°, +67°] | −1 | 67 | 67 |
| elbow | θ₂ ∈ [0°, 180°] | −1 | 180 | 180 |
| wrist | θ₃ ∈ [−135°, +45°] | −1 | 45 | 45 |

Keeping the kinematic frame and the servo commands separate means re-indexing a
horn is a one-number config change, never a change to the equations — the same
separation a URDF + driver layer uses on production robots. **Home** is θ₂ = 90°
(not the zero pose), deliberately avoiding the full-extension singularity where
the arm is ill-conditioned.

---

## 6. Smoothed motion

The MG996R has an internal proportional controller and no speed input: hand it a
distant target and it slews at full rate, then stops hard — the jerk logged in
§4. The fix is never to command a large step. The controller streams targets at
a fixed 50 Hz, each ~1 mm/1° from the last, with acceleration-limited velocity
ramps and task-space (straight-line) interpolation. Smooth motion also improves
the quality of teleoperated demonstrations collected later.

---

## 7. Backlash

Hobby-servo gear trains have meaningful free play, which at full extension maps
to millimeters of tip error. Rather than buy feedback servos, a **spring preload**
on the gear train holds the gears against one face, removing most of the play —
a ~$0 fix reproducible by anyone building on hobby servos. (Migrating to
serial-bus feedback servos remains the documented upgrade path if precision
later demands it.)

---

## 8. Workspace optimization

Two distinct regions matter:

- **Reachable workspace** — everywhere the tip can be placed at *some*
  orientation.
- **Dexterous workspace** — where the wrist can *also* hold the camera at a
  fixed orientation. Because angles sum along the chain, the wrist must absorb
  `span(θ₁)+span(θ₂)`, so the dexterous region is a subset of the reachable one.

A small interactive tool (matplotlib sliders) was written to place the servo
horns and the patient target zone, optimizing for the **largest dexterous area
in quadrants 3–4** (in front of the arm) while:

- avoiding both the full-extension and fully-folded singularities,
- treating the mounting bracket as a physical obstacle (link–bracket collision
  excluded), and
- keeping the well-conditioned region (near θ₂ = 90°, maximum Yoshikawa
  manipulability `w = l₁·l₂·|sin θ₂|`) over the patient.

The chosen placement holds the camera orientation across ~80% of the reachable
area and covers 100% of the intended eye-zone.

---

## 9. Open problems & next steps

- **Hardware ROS 2 node** — a servo driver node on the Pi, subscribing to
  `/joint_states`, to connect the ROS 2 graph to the physical arm.
- **URDF + RViz** — declarative description and visualization.
- **Perception** — eye/target detection from the in-hand camera, published as
  the control target.
- **Learned control** — teleoperated data collection → LeRobot dataset →
  imitation-learning policy (ACT) → on-hardware deployment, with a quantitative
  evaluation protocol (success rate across randomized target poses) and a
  failure-mode writeup.
- **Precision ceiling** — if the preloaded hobby servos plateau, migrate to
  serial-bus feedback servos (true position feedback) and re-measure.