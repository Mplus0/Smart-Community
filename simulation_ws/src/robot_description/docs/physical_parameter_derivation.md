# Physical-parameter derivation for v0.5

## 1. Coordinate normalization

Official STL wheel geometry:

- CAD wheel-centre Z: `0.035791123 m`
- wheel radius from STL envelope: `0.037230282 m`
- lowest wheel point in CAD: approximately `-0.001439159 m`

Therefore `base_footprint` is placed at the true contact plane and `base_link` is one wheel radius above it:

```text
base_footprint -> base_link z = 0.037230282 m
```

The official visual remains aligned with:

```text
mesh visual z = -0.035791123 m relative to base_link
```

## 2. Lower chassis collision

Simplified lower-body collision recovered from the STL envelope:

```text
size   = (0.310999990, 0.175999999, 0.052999999) m
origin = (0.009499997, 0, 0.009300001) m relative to base_link
```

The dense official STL is visual-only.

## 3. Wheel geometry

```text
radius = 0.037230282 m
width  = 0.032000005 m
FL = (+0.106000002, +0.125000000, 0)
FR = (+0.106000002, -0.125000000, 0)
RL = (-0.106000002, +0.125000000, 0)
RR = (-0.106000002, -0.125000000, 0)
```

## 4. Official mass correction

The old SolidWorks export reported only:

```text
0.486218814966626 kg
```

The official hardware parameter table supplied later states the real robot empty mass is:

```text
5.2 kg
```

Therefore the old CAD mass is no longer used as the physical total.

Individual wheel masses have not been measured. v0.5 preserves the previous CAD-derived relative split and scales it uniformly by:

```text
5.2 / 0.486218814966626 = 10.69477329946
```

Resulting simulation baseline:

```text
each wheel = 0.160421599492 kg
base body  = 4.558313602032 kg
total      = 5.200000000000 kg
```

This is a traceable approximation, not a claim that each real wheel has that measured mass.

## 5. Inertia scaling

Because the geometry is unchanged, the v0.3 inertia tensors are scaled linearly with the same mass factor.

Wheel inertia, axis Y:

```text
Ixx = Izz = 0.000069279169 kg*m^2
Iyy       = 0.000111179698 kg*m^2
```

Base inertia:

```text
ixx = 0.004667882197
ixy = -0.000385595025
ixz = -0.003250932891
iyy = 0.031488485020
iyz = 0.000491303319
izz = 0.031491967442
```

The base COM location is retained from the previous CAD-derived baseline because no measured real COM is available yet.

## 6. Official speed correction

Official maximum chassis speed:

```text
v_max = 0.6 m/s
```

With STL wheel radius `r = 0.037230282 m`:

```text
omega_max = v_max / r
          = 16.115913385 rad/s
```

This value replaces the earlier temporary 30 rad/s limit in both the URDF wheel joints and the command-controller default.

## 7. Parameters still deferred

The following still require Linux/Gazebo or real-hardware verification:

- individual wheel masses;
- real center of mass;
- motor torque and RPM;
- gearbox reduction;
- encoder resolution;
- `mu1`, `mu2`, `fdir1` tuning for the mecanum rollers;
- joint effort limit;
- acceleration limits;
- controller gains.
