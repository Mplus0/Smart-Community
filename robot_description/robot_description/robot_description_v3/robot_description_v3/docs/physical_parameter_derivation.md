# Physical-parameter derivation for v0.3

## 1. Coordinate normalization

Official STL wheel geometry:

- CAD wheel-centre Z: `0.035791123 m`
- wheel radius from envelope: `0.037230282 m`
- lowest wheel point in CAD: approximately `-0.001439159 m`

Therefore `base_footprint` is placed at the true contact plane and `base_link` is placed one wheel radius above it:

```text
base_footprint -> base_link z = 0.037230282 m
```

The official visual remains aligned by retaining the mesh offset:

```text
mesh visual z = -0.035791123 m relative to base_link
```

This raises the complete CAD visual by about `1.439 mm` compared with v0.2, exactly compensating for the CAD wheel geometry that extended below the original CAD Z=0 plane.

## 2. Lower chassis collision

The lower chassis structural components in the official STL give the combined envelope:

```text
X: -0.146000 .. +0.165000 m
Y: -0.088000 .. +0.088000 m
Z: +0.018591 .. +0.071591 m (CAD coordinates)
```

Equivalent simplified collision box:

```text
size   = (0.310999990, 0.175999999, 0.052999999) m
origin = (0.009499997, 0, 0.009300001) m relative to normalized base_link
```

This box deliberately represents the lower body only. It avoids using the very dense official STL for collision calculation.

## 3. Wheel geometry

```text
radius = 0.037230282 m
width  = 0.032000005 m
FL = (+0.106000002, +0.125000000, 0) relative to base_link
FR = (+0.106000002, -0.125000000, 0)
RL = (-0.106000002, +0.125000000, 0)
RR = (-0.106000002, -0.125000000, 0)
```

Each wheel collision is a cylinder whose axis is Y.

## 4. Mass split

Official lumped model:

```text
total mass = 0.486218814966626 kg
```

A pure STL-volume apportionment would give about `0.01931 kg` per wheel, but subtracting the corresponding wheel inertias and parallel-axis terms from the official lumped inertia makes the residual base inertia non-positive. This demonstrates that the exported lumped inertia cannot be cleanly decomposed by mesh volume alone.

v0.3 therefore uses a conservative numerical baseline:

```text
wheel mass = 0.015000 kg each
base mass  = 0.426218814966626 kg
sum        = 0.486218814966626 kg
```

The total official mass is preserved.

## 5. Wheel inertia

Using a solid-cylinder approximation, axis Y:

```text
Iyy = 1/2 m r^2 = 1.039570423350e-05 kg*m^2
Ixx = Izz = 1/12 m (3r^2 + w^2) = 6.477852516748e-06 kg*m^2
```

## 6. Residual base inertial baseline

After subtracting four `0.015 kg` wheel approximations from the official lumped mass properties:

```text
base COM relative to normalized base_link:
(-0.081965593727, 0.008421364598, 0.103109011698) m

base inertia (kg*m^2):
ixx = 0.000436463875
ixy = -0.000036054530
ixz = -0.000303973988
iyy = 0.002944287283
iyz = 0.000045938638
izz = 0.002944612902
```

Eigenvalues of the residual inertia are positive (approximately `3.998e-4`, `2.910e-3`, `3.016e-3`), so the baseline is numerically valid.

## 7. Parameters deliberately deferred

v0.3 does **not** claim final values for:

- real wheel mass/material distribution;
- motor torque and maximum RPM;
- gearbox reduction;
- encoder resolution;
- mecanum roller direction/contact anisotropy;
- wheel-ground friction tuned for lateral motion;
- controller gains and effort limits.

These belong to the controller/dynamics calibration step after the static-contact model is verified.
