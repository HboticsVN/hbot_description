# Building the HBOT model from CAD meshes

This guide explains, step by step, how `hbot.urdf.xacro` is built from the two
CAD exports in [`models/`](../models): `base_link.stl` (chassis + caster +
lidar) and `wheel.stl` (one drive wheel). One xacro produces **two robot
descriptions**: a simple `hbot.urdf` for the real robot (SLAM/Nav2) and a full
`hbot_sim.urdf` for Gazebo (Step 7). The guide also covers what to do when the
CAD changes.

Branches: `feat/cad-model` in `hbot_description` (from `main` @ `c6ccb37`),
`hbot_simulation` and `hbot_bringup`.

---

## 0. Result at a glance

| Item | Value | Source |
|---|---|---|
| Frame origin (`base_footprint` ≡ `base_link`) | on the ground, under the drive-axle mid-point | convention |
| Wheel Ø × width | 67.4 × 30.6 mm | CAD |
| Wheel track (`wheel_separation`) | **0.195 m** | your spec (190–200 mm); CAD has 0.183 m |
| Caster ball | r 11.6 mm at x +105 mm, y +10 mm | CAD |
| Lidar | YDLidar X3, `laser` at x +42.5 mm, z 136.8 mm (scan plane) | CAD |
| Lidar sim | 8 Hz, 375 samples, 0.12–8 m | X3 datasheet (3 kHz / 8 Hz) |
| Chassis collision box | x −0.050…+0.132, y ±0.090, z 0.024…0.117 m | CAD |

```
                 +x (forward)
                   ^
          caster o |            laser (+0.0425, 0, 0.137)
                   |    [X3]
   left wheel ||   +---->  ...  || right wheel      y: +0.0975 / -0.0975
                   |
             base_footprint (0,0,0) = axle mid-point on the ground
```

Files:

| File | Role |
|---|---|
| `models/*.stl` | raw CAD exports (mm, CAD frame); the **input**, don't edit by hand |
| `scripts/prepare_meshes.py` | turns `models/` into `meshes/` + `urdf/cad_params.xacro` |
| `meshes/{base_link,wheel,lidar}.stl` | link-frame meshes in metres (generated) |
| `urdf/cad_params.xacro` | measured dimensions as xacro properties (generated) |
| `urdf/hbot.urdf.xacro` | entry point: the shared TF frames; `sim:=true` adds the rest |
| `urdf/hbot_body.xacro` | sim only: mesh visuals, collisions, inertias, wheels, caster |
| `urdf/hbot.gazebo.xacro` | sim only: lidar/IMU sensors, diff-drive, friction |
| `cmake/generate_urdf.cmake` | build step that writes the URDFs below |
| `urdf/hbot.urdf` | **real robot** (generated): 4 frames, no geometry |
| `urdf/hbot_sim.urdf`, `urdf/hbot_sim.sdf` | **Gazebo** (generated): the full model |

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

## Step 6: Write the xacro

`urdf/hbot.urdf.xacro` includes `cad_params.xacro` and uses the measured
values, so no dimension is copied in by hand. Design choices:

1. **Visuals = meshes, collisions = primitives.** Box for the chassis,
   cylinders for the wheels and lidar, a sphere for the caster. A 58k-triangle
   mesh as a collision shape would slow Gazebo down for no benefit.
2. **Wheel track is overridden** to `wheel_separation = 0.195` (your 190–200 mm
   spec) instead of the CAD's 0.183. The wheel meshes move outward to match.
3. **The caster link has no visual.** The ball is already in `base_link.stl`; the
   link only carries the frictionless contact sphere.
4. **`laser` is the lidar frame**, the same `frame_id` the ydlidar driver stamps
   on `/scan` (`ydlidar_ros2_driver/params/ydlidar_x3.yaml`).
5. **Gazebo-only content lives in `hbot.gazebo.xacro`**, and the physical body
   in `hbot_body.xacro` (ROS convention: keep the description usable without
   Gazebo). Both are only included with `sim:=true` (Step 7).
6. **Masses** are kept close to the previous box model (base 0.5, wheels 0.1,
   lidar 0.1 kg) so the tuned sim driving doesn't change.

YDLidar X3 simulation (`hbot.gazebo.xacro`): 3 kHz sampling ÷ 8 Hz ⇒ 375
samples/rev, range 0.12–8 m, 1 cm Gaussian noise.

## Step 7: Two outputs from one xacro, `hbot.urdf` (real) and `hbot_sim.urdf` (sim)

SLAM (Cartographer / slam_toolbox) only uses the URDF's **static TF**:
`base_footprint → base_link → laser` (+ `imu_link`). The odometry transform
`odom → base_footprint` comes from the driver/EKF. Everything else is only
needed by Gazebo, and on the real robot it causes problems: the driver
publishes no `/joint_states`, so continuous wheel joints would have no TF and
RViz/tf2 would keep reporting "No transform to left_wheel_link".

So the xacro takes one argument, `sim` (default `false`):

```xml
<xacro:arg name="sim" default="false"/>
<xacro:property name="sim" value="$(arg sim)"/>

<link name="base_link">
  <xacro:if value="${sim}"><xacro:base_body/></xacro:if>   <!-- empty on the real robot -->
</link>
...
<xacro:if value="${sim}">
  <xacro:wheel .../> <xacro:caster/> <xacro:include filename="hbot.gazebo.xacro"/>
</xacro:if>
```

| | `hbot.urdf` (real, `sim:=false`) | `hbot_sim.urdf` (Gazebo, `sim:=true`) |
|---|---|---|
| Links | `base_footprint`, `base_link`, `laser`, `imu_link` | + `left/right_wheel_link`, `front_caster` |
| Visuals / collisions / inertias | none | CAD meshes, primitives, estimates |
| Gazebo sensors + diff-drive | none | yes |
| Laser pose | **identical**: both come from `cad_params.xacro` | |
| Used by | `hbot_bringup` (`robot_state_publisher` on the Pi) | `hbot_simulation/launch/hbot_house.launch.py` |

The real robot's frames can't drift from the sim's, because they're the same
lines of xacro. The frames are declared once, and the sim only adds content inside them.

Build plumbing:

- `cmake/generate_urdf.cmake` runs at install time:
  `xacro hbot.urdf.xacro -o hbot.urdf` and
  `xacro hbot.urdf.xacro sim:=true -o hbot_sim.urdf`. A xacro error **fails the
  build** (the old inline command failed silently). `hbot_sim.sdf` is
  produced only where `gz` exists, so the Pi's arm64 image (no Gazebo) still
  builds.
- `CMakeLists.txt`: calls that script, then installs `launch meshes rviz urdf`.
- `package.xml`: export `<gazebo_ros gazebo_model_path="${prefix}/.."/>` so
  Gazebo can resolve `package://hbot_description/meshes/...` (it becomes
  `model://hbot_description/...` in the SDF).
- `launch/hbot_description.launch.py`: new `sim` arg passed to xacro
  (`sim:=false` shows the real frames, `sim:=true` the full model). The
  `robot_description` is wrapped in `ParameterValue(..., value_type=str)`;
  without it, Humble's launch tries to parse the xacro output as YAML and fails.

Consumers (other submodules, branch `feat/cad-model`):

- `hbot_simulation/launch/hbot_house.launch.py` reads `urdf/hbot_sim.urdf`.
- `hbot_bringup/launch/hbot_bringup.launch.py` reads
  `hbot_description/urdf/hbot.urdf`. The hand-written
  `hbot_bringup/config/hbot.urdf` is deleted (and dropped from `setup.py`), and
  `hbot_bringup/package.xml` gains `<exec_depend>hbot_description</exec_depend>`.
  `hbot_description` was already part of the Pi build, so nothing new ships.

Check the real one yourself: it should contain exactly 4 links and 3 fixed
joints.

```bash
check_urdf src/hbot_description/urdf/hbot.urdf
```

## Step 8: Validate

```bash
# 1. Build: generates urdf/hbot.urdf, urdf/hbot_sim.urdf, urdf/hbot_sim.sdf
./build_packages.sh hbot_description
bash -c 'source /opt/ros/humble/setup.bash && cd src/hbot_description/urdf \
  && check_urdf hbot.urdf && check_urdf hbot_sim.urdf'

# 2. View in RViz: real frames, then the full model
source install/setup.bash
ros2 launch hbot_description hbot_description.launch.py rviz:=true             # sim:=false
ros2 launch hbot_description hbot_description.launch.py rviz:=true sim:=true
#   (with sim:=true and no Gazebo the wheels show "No transform": nothing
#    publishes their joint states in this launch)

# 3. Headless Gazebo (use a private domain: this publishes /cmd_vel!)
ROS_DOMAIN_ID=42 ./scripts/dev_sim_smoke.sh 35
```

Results on 2026-09-23:

- `check_urdf hbot.urdf`: `base_footprint → base_link → {imu_link, laser}`.
- `check_urdf hbot_sim.urdf`: `base_footprint → base_link → {front_caster,
  imu_link, left_wheel_link, laser, right_wheel_link}`; SDF conversion OK.
- `robot_state_publisher` with `sim:=false` and with `sim:=true`: both give
  `base_footprint→laser (0.043, 0, 0.137)`.
- Gazebo: `/scan` at 8.0 Hz, 375 samples, `range_min 0.12`, `range_max 8.0`,
  nearest return 1.4 m, i.e. **no self-hits**.
- TF: `base_link→laser (0.043, 0, 0.137)`, `→left_wheel (0, 0.098, 0.034)`,
  `→front_caster (0.105, 0.010, 0.012)`.
- Driving: 0.15 m/s forward moves the robot ≈0.7 m in ~4.5 s; spinning works.
- Known (pre-existing): the idle robot creeps ≈1 mm/s in Gazebo. The old box
  model does the same, so it isn't introduced by this change.

> Note: `build/hbot_simulation` in this workspace is owned by root (from a Docker
> build), so `./build_packages.sh hbot_simulation` fails with *Permission
> denied*. Fix with `sudo chown -R $USER build install log`, or build into a
> separate `--build-base/--install-base`.

## Step 9: When the CAD changes

1. Re-export `base_link.stl` and `wheel.stl` (binary STL, mm, same assembly
   frame for both) into `models/`.
2. `python3 scripts/prepare_meshes.py --preview ../../log/hbot_cad_preview.png`
   and check the preview.
3. If the lidar moved or changed size, adjust `LIDAR_BOX` in the script.
4. Rebuild (this regenerates `urdf/hbot.urdf`, `urdf/hbot_sim.urdf` and
   `urdf/hbot_sim.sdf`) and repeat Step 8.

## Follow-ups outside this package

The Pi's laser pose is now fixed: it comes from `hbot.urdf` (x 0.0425, z 0.137
instead of the old hand-written 0.08 / 0.14). **Re-check SLAM on the real robot
after deploying.** If the scan looks rotated 180°, set the yaw on
`lidar_joint` in `hbot.urdf.xacro`. The X3 driver runs with
`inverted: true`, and the old Pi URDF had a commented-out `rpy="0 0 3.14"`.

These `hbot_bringup` values still use the old guesses:

| Where | Now | From CAD |
|---|---|---|
| `config/yahboom_driver_params.yaml` `wheel_track` | 0.20 | 0.195 (calibrate: 190–200) |
| `config/yahboom_driver_params.yaml` `wheel_diameter` | 0.065 | 0.0674 (tyre in CAD) |
| `config/nav2_params.yaml` `footprint` | ±0.09 x, ±0.12 y (centred) | x −0.05…+0.132, y ±0.113 (the robot extends forward of the axle) |
