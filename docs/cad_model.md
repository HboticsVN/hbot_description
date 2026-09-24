# Building the HBOT model from CAD meshes

This guide explains, step by step, how the CAD exports in
[`models/`](../models) (`base_link.stl`: chassis + caster + lidar, and
`wheel.stl`: one drive wheel) become the link-frame meshes in `meshes/` and the
measured dimensions in `urdf/cad_params.xacro`. It also covers what to do when
the CAD changes.

How those meshes and dimensions are assembled into the robot description
(xacro layout, `config/hbot_geometry.yaml`, the real vs. Gazebo variants,
launch files, tests) is in [`robot_description.md`](robot_description.md).

---

## 0. Result at a glance

| Item | Value | Source |
|---|---|---|
| Frame origin (`base_footprint` ≡ `base_link`) | on the ground, under the drive-axle mid-point | convention |
| Wheel Ø × width | 67.4 × 30.6 mm | CAD |
| Wheel track | CAD 0.183 m; the model uses **0.190 m** (measured, `config/hbot_geometry.yaml`) | measured |
| Caster ball | r 11.6 mm at x +105 mm, y +10 mm | CAD |
| Lidar | YDLidar X3, `laser` at x +42.5 mm, z 136.8 mm (scan plane) | CAD |
| Chassis collision box | x −0.050…+0.132, y ±0.090, z 0.024…0.117 m | CAD |

```
                 +x (forward)
                   ^
          caster o |            laser (+0.0425, 0, 0.137)
                   |    [X3]
   left wheel ||   +---->  ...  || right wheel      y: +0.095 / -0.095
                   |
             base_footprint (0,0,0) = axle mid-point on the ground
```

Files covered by this guide:

| File | Role |
|---|---|
| `models/*.stl` | raw CAD exports (mm, CAD frame); the **input**, don't edit by hand |
| `scripts/prepare_meshes.py` | turns `models/` into `meshes/` + `urdf/cad_params.xacro` |
| `meshes/{base_link,wheel,lidar}.stl` | link-frame meshes in metres (generated, committed: the URDF loads them) |
| `urdf/cad_params.xacro` | measured dimensions as xacro properties (generated, committed) |

---

## Step 1: Inspect the raw STLs

Before using a CAD mesh in a URDF you need three facts about it: its **units**,
its **coordinate frame**, and **what it contains**.

```bash
python3 - <<'EOF'
import numpy as np
d = open("models/base_link.stl", "rb").read()
n = int(np.frombuffer(d[80:84], "<u4")[0])
v = np.frombuffer(d[84:], dtype=[("n","<3f4"),("v","<9f4"),("a","<u2")], count=n)["v"].reshape(-1,3)
print(n, "triangles; min", v.min(0), "max", v.max(0))
EOF
```

What this showed:

- **Binary STL, millimetres.** The chassis bounding box is ~230 × 229 × 145.
  URDF works in metres, so everything gets scaled by 0.001.
- **Not in a robot frame.** The coordinates are around (280, −227, 113), far from
  the origin. The ~square bounding box for a non-square robot means the
  model is also **rotated about Z**. The wheel looks round from every axis,
  which confirms its axle isn't aligned with X or Y.
- **`base_link.stl` also holds the caster ball and the lidar.** Only the drive
  wheel is a separate file (one wheel; the other is its mirror).

## Step 2: Find the CAD yaw from the wheel axle

A wheel is mostly two large flat side faces, so if you add up the face normals
weighted by area (`Σ area·n·nᵀ`), one eigenvector stands out: the **axle
direction**. For this CAD it points 37.4° from the X axis (the base walls'
normals agree), so the whole assembly is yawed. The body frame is then:

- `y` (left)  = −axle direction, oriented so the exported wheel is the **left** one
- `x` (forward) = z × axle, oriented toward the caster (the chassis extends
  132 mm in front of the axle and only 50 mm behind it)
- `z` = up (the axle is horizontal, which the script checks)

This is a proper rotation (determinant +1). The script builds it, so no mesh
gets mirrored.

## Step 3: Pick the origin (`base_footprint`)

REP-105 puts `base_footprint` on the ground under the robot's rotation
centre. For a differential drive that is the **mid-point of the drive axle**:

- **ground** = the lowest point of the wheel (it matches the caster bottom, so the
  CAD sits level),
- **axle x / z** = the wheel's centre in the side view,
- **y = 0** at the chassis symmetry plane. Reflecting the chassis about that plane
  matches it to within 0.05 mm, and the lidar head centre lies on it too.

`base_link` stays coincident with `base_footprint`, as it always has on the real
robot and in the Nav2/driver configs.

## Step 4: Measure the parts

All of these are computed by the script in the body frame:

- **Wheel:** radius = half its height (33.7 mm); width along the axle (30.6 mm);
  tyre mid-plane = the mean `y` of the points at maximum radius. In the CAD
  the wheels sit at a track of 182.7 mm.
- **Caster:** the only chassis part touching the floor. The fit is seeded from the
  floor-contact points, then a sphere is fitted to the ball's lower half (the
  bracket posts around it are excluded) → r 11.6 mm at (105, 10) mm.
  It is ~10 mm left of centre in the CAD; that's kept as modelled.
- **Lidar:** the D-shaped X2/X3 housing with its round head on the top plate.
  A circle fit on the head's top rim gives its centre (x 42.5 mm, y 0,
  Ø 63 mm). The scan plane is set 8 mm below the head top (the optical
  window is midway up the rotating turret) → z 136.8 mm. Tune
  `SCAN_PLANE_BELOW_TOP` in the script if you measure the real height.
- **Chassis collision box:** the bounds of the chassis without the caster and lidar.
  Its top (0.117 m) must stay **below the scan plane** (0.137 m), otherwise
  the simulated lidar sees the robot's own body as a ring of obstacles.

## Step 5: Split and re-export the meshes in link frames

Each mesh is written in **its own link frame**, in metres, so the URDF
`<visual>` origins are all zero:

| Mesh | Frame | Notes |
|---|---|---|
| `meshes/base_link.stl` | `base_link` | chassis + caster ball; lidar removed |
| `meshes/lidar.stl` | `laser` | triangles inside the lidar box, above the top plate |
| `meshes/wheel.stl` | wheel centre, axle on +y | the left wheel; the right uses the same mesh turned 180° about z |

Run it (numpy only; matplotlib for the preview):

```bash
cd src/hbot_description
python3 scripts/prepare_meshes.py --preview ../../log/hbot_cad_preview.png
```

It prints every measured value and writes `urdf/cad_params.xacro`. **Always
look at the preview** (top / side / front views, the red × is `laser`) to
check that x points to the caster, the lidar (orange) is split cleanly, and the
wheels are on the correct sides.


## Step 6: Hand the measurements to the description

`cad_params.xacro` only defines properties (`cad_wheel_radius`,
`cad_laser_x`, ...). They reach the model through
[`config/hbot_geometry.yaml`](../config/hbot_geometry.yaml): every entry set to
`cad` takes the matching CAD value, and a number overrides it with a
measurement from the real robot. The wheel track is such an override (0.190 m
measured vs. 0.183 m in the CAD). The chassis collision box and the lidar
cylinder always come straight from the CAD bounds.

Design choices made at this step (details in
[`robot_description.md`](robot_description.md)):

1. **Visuals = meshes, collisions = primitives.** Box for the chassis,
   cylinders for the wheels and lidar, a sphere for the caster. A 58k-triangle
   mesh as a collision shape would slow Gazebo down for no benefit.
2. **The caster link has no visual.** The ball is already in `base_link.stl`;
   the link only carries the contact sphere.
3. **`laser` is the lidar frame**, the same `frame_id` the ydlidar driver stamps
   on `/scan` (`ydlidar_ros2_driver/params/ydlidar_x3.yaml`).

## Step 7: When the CAD changes

1. Re-export `base_link.stl` and `wheel.stl` (binary STL, mm, same assembly
   frame for both) into `models/`.
2. `python3 scripts/prepare_meshes.py --preview ../../log/hbot_cad_preview.png`
   and check the preview.
3. If the lidar moved or changed size, adjust `LIDAR_BOX` in the script.
4. Review `config/hbot_geometry.yaml`: an override (e.g. `wheels.track`) is not
   updated by the script.
5. Rebuild and test, see [`robot_description.md`](robot_description.md) Step 6:

   ```bash
   ./build_packages.sh hbot_description
   colcon test --packages-select hbot_description --event-handlers console_direct+
   ```
