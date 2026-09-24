#!/usr/bin/env python3
"""Convert the raw CAD exports in models/ into URDF-ready meshes.

The CAD assembly is exported in millimetres, yawed ~37 deg about Z and offset
far from the origin, with the lidar baked into the chassis. This script:

  1. detects the CAD yaw from the wheel's axle (the dominant face-normal axis),
  2. re-expresses everything in the ROS body frame (REP-103: x forward toward
     the caster, y left, z up) with the origin on the ground under the
     mid-point of the drive axle (= base_footprint / base_link),
  3. splits the lidar out of the chassis into its own mesh,
  4. writes binary STLs in metres, each in its own link frame:
       meshes/base_link.stl  - chassis,      frame = base_link
       meshes/wheel.stl      - left wheel,   frame = wheel centre, axle on +y
       meshes/lidar.stl      - lidar,        frame = laser (head centre, scan plane)
  5. writes urdf/cad_params.xacro with the dimensions measured from the CAD
     so the xacro never carries hand-copied numbers.

Re-run it whenever models/*.stl are re-exported:

    python3 scripts/prepare_meshes.py [--preview preview.png]

Only numpy is required (matplotlib for --preview).
"""
import argparse
import os

import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Lidar bounding region in the body frame (mm). Everything of the chassis mesh
# whose triangle centroid falls inside goes to lidar.stl. The lidar sits on
# the top plate (top plate surface ~108 mm; lidar body starts ~115 mm).
LIDAR_BOX = {'x': (-10.0, 125.0), 'y': (-45.0, 45.0), 'z_min': 112.0}
# The optical window of the YDLidar X2/X3 head is roughly midway up the
# rotating turret, ~8 mm below its top face.
SCAN_PLANE_BELOW_TOP = 8.0


def load_stl(path):
    data = open(path, 'rb').read()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    if len(data) != 84 + 50 * n:
        raise SystemExit(f'{path}: not a binary STL (ASCII STL is not supported)')
    rec = np.frombuffer(data[84:], dtype=np.dtype(
        [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
    return rec['v'].reshape(-1, 3, 3).astype(np.float64)


def save_stl(path, tris_m, name):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tris = tris_m.astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=np.dtype(
        [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'] = nrm
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def face_normals(tris):
    c = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    area = np.linalg.norm(c, axis=1) / 2
    return c / (2 * area[:, None] + 1e-12), area


def fit_circle(xy):
    a = np.c_[2 * xy, np.ones(len(xy))]
    c = np.linalg.lstsq(a, (xy ** 2).sum(1), rcond=None)[0]
    return c[:2], np.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)


def fit_sphere(p):
    a = np.c_[2 * p, np.ones(len(p))]
    c = np.linalg.lstsq(a, (p ** 2).sum(1), rcond=None)[0]
    return c[:3], np.sqrt(c[3] + (c[:3] ** 2).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--preview', help='write a 3-view PNG of the result (needs matplotlib)')
    args = ap.parse_args()

    base = load_stl(os.path.join(PKG, 'models', 'base_link.stl'))
    wheel = load_stl(os.path.join(PKG, 'models', 'wheel.stl'))

    # 1. Axle direction: a wheel is mostly two flat side faces, so the
    # area-weighted normal tensor has one distinct (largest) eigenvector.
    n, area = face_normals(wheel)
    _, vec = np.linalg.eigh(np.einsum('i,ij,ik->jk', area, n, n))
    axle = vec[:, 2]
    if abs(axle[2]) > 0.05:
        raise SystemExit(f'wheel axle is not horizontal in the CAD frame: {axle}')
    axle = np.array([axle[0], axle[1], 0.0])
    axle /= np.linalg.norm(axle)
    # Point `axle` from the wheel toward the chassis centre, so the exported
    # wheel is the LEFT one once body y = -axle.
    if (base.reshape(-1, 3).mean(0) - wheel.reshape(-1, 3).mean(0)) @ axle < 0:
        axle = -axle
    fwd = np.array([-axle[1], axle[0], 0.0])        # z x axle
    rot = np.vstack([fwd, -axle, [0.0, 0.0, 1.0]])  # CAD -> body, det = +1
    yaw_deg = np.degrees(np.arctan2(fwd[1], fwd[0]))

    bp = base.reshape(-1, 3) @ rot.T
    wp = wheel.reshape(-1, 3) @ rot.T

    # 2. Reference points, all in the rotated (still millimetre) frame.
    ground = wp[:, 2].min()
    axle_x = (wp[:, 0].min() + wp[:, 0].max()) / 2
    axle_z = (wp[:, 2].min() + wp[:, 2].max()) / 2
    wheel_radius = (wp[:, 2].max() - wp[:, 2].min()) / 2
    r = np.hypot(wp[:, 0] - axle_x, wp[:, 2] - axle_z)
    wheel_y = wp[r > 0.98 * r.max(), 1].mean()      # tyre mid-plane
    wheel_width = wp[:, 1].max() - wp[:, 1].min()
    mid_y = (bp[:, 1].min() + bp[:, 1].max()) / 2   # chassis symmetry plane

    origin = np.array([axle_x, mid_y, ground])
    base_b = base @ rot.T - origin                  # body frame, mm
    if base_b[..., 0].max() < -base_b[..., 0].min():
        raise SystemExit('chassis extends further behind the axle than in front; '
                         'the caster-forward convention does not hold for this CAD')

    # 3. Split the lidar off the chassis.
    cen = base_b.mean(1)
    in_lidar = ((cen[:, 0] > LIDAR_BOX['x'][0]) & (cen[:, 0] < LIDAR_BOX['x'][1]) &
                (cen[:, 1] > LIDAR_BOX['y'][0]) & (cen[:, 1] < LIDAR_BOX['y'][1]) &
                (cen[:, 2] > LIDAR_BOX['z_min']))
    lidar_b, chassis_b = base_b[in_lidar], base_b[~in_lidar]
    lp = lidar_b.reshape(-1, 3)
    lidar_top = lp[:, 2].max()
    (lidar_x, lidar_y), head_r = fit_circle(lp[lp[:, 2] > lidar_top - 3.0, :2])
    laser = np.array([lidar_x, lidar_y, lidar_top - SCAN_PLANE_BELOW_TOP])

    # Caster ball: the only chassis part touching the floor. Seed its centre
    # from the floor-contact points, then fit a sphere to the lower part of
    # the ball (the bracket posts around it stop higher up).
    cp = chassis_b.reshape(-1, 3)
    seed = cp[cp[:, 2] < 3.0, :2].mean(0)
    low = cp[(cp[:, 2] < 20.0) & (np.linalg.norm(cp[:, :2] - seed, axis=1) < 20.0)]
    caster_c, caster_r = fit_sphere(low)
    caster_r = caster_c[2]                          # tangent to the floor

    # Chassis collision box: everything but the caster ball and the lidar.
    body = cp[np.linalg.norm(cp - caster_c, axis=1) > caster_r + 2.0]
    body = body[body[:, 2] > caster_c[2] + caster_r + 1.0]
    bmin, bmax = body.min(0), body.max(0)

    # 4. Meshes, each in its own link frame, in metres.
    wheel_b = wheel @ rot.T - np.array([axle_x, wheel_y, axle_z])
    save_stl(os.path.join(PKG, 'meshes', 'base_link.stl'), chassis_b / 1000, 'hbot base_link')
    save_stl(os.path.join(PKG, 'meshes', 'wheel.stl'), wheel_b / 1000, 'hbot wheel (left)')
    save_stl(os.path.join(PKG, 'meshes', 'lidar.stl'), (lidar_b - laser) / 1000, 'hbot lidar')

    # 5. Measured dimensions for the xacro.
    params = {
        'cad_wheel_radius': wheel_radius,
        'cad_wheel_width': wheel_width,
        'cad_wheel_separation': 2 * (wheel_y - mid_y),
        'cad_caster_x': caster_c[0], 'cad_caster_y': caster_c[1],
        'cad_caster_radius': caster_r,
        'cad_laser_x': laser[0], 'cad_laser_y': laser[1], 'cad_laser_z': laser[2],
        'cad_lidar_head_radius': head_r,
        'cad_lidar_height': lidar_top - lp[:, 2].min(),
        'cad_body_min_x': bmin[0], 'cad_body_max_x': bmax[0],
        'cad_body_min_y': bmin[1], 'cad_body_max_y': bmax[1],
        'cad_body_min_z': bmin[2], 'cad_body_max_z': bmax[2],
    }
    lines = ['<?xml version="1.0"?>',
             '<!-- GENERATED by scripts/prepare_meshes.py from models/*.stl - do not edit.',
             f'     CAD yaw {yaw_deg:.2f} deg; all values in metres, body frame',
             '     (origin on the ground under the drive-axle mid-point). -->',
             '<robot xmlns:xacro="http://ros.org/wiki/xacro">']
    lines += [f'  <xacro:property name="{k}" value="{v / 1000:.4f}"/>' for k, v in params.items()]
    lines.append('</robot>')
    with open(os.path.join(PKG, 'urdf', 'cad_params.xacro'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print(f'CAD yaw: {yaw_deg:.2f} deg')
    print(f'triangles: chassis {len(chassis_b)}, lidar {len(lidar_b)}, wheel {len(wheel_b)}')
    for k, v in params.items():
        print(f'  {k:24s} {v / 1000:8.4f} m')

    if args.preview:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        wl = wheel_b.reshape(-1, 3) + np.array([0, params['cad_wheel_separation'] / 2, wheel_radius])
        wr = wl * np.array([1, -1, 1])
        parts = [(cp, 'tab:blue'), (lp, 'tab:orange'), (wl, 'k'), (wr, 'k')]
        fig, axs = plt.subplots(1, 3, figsize=(18, 6))
        for ax, (i, j, title) in zip(axs, [(0, 1, 'top (x fwd, y left)'),
                                           (0, 2, 'side (x, z)'), (1, 2, 'front (y, z)')]):
            for p, c in parts:
                ax.scatter(p[::7, i], p[::7, j], s=0.05, c=c)
            ax.scatter([laser[i]], [laser[j]], c='r', marker='x', s=80)
            ax.set_aspect('equal')
            ax.grid(True)
            ax.set_title(title + ' [mm]')
        axs[2].invert_xaxis()   # look from the front: +y (left) on the right of the image
        plt.savefig(args.preview, dpi=80)
        print(f'preview -> {args.preview}')


if __name__ == '__main__':
    main()
