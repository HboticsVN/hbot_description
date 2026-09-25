"""Checks for the HBOT description (run with `colcon test`).

They catch the ways the real-robot and the Gazebo descriptions, or the
description and the driver, can drift apart. See docs/robot_description.md,
"Step 6: Test".
"""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(PKG, 'urdf', 'hbot.urdf.xacro')
GEOMETRY = os.path.join(PKG, 'config', 'hbot_geometry.yaml')
TOL = 1e-6


def expand(**args):
  doc = xacro.process_file(XACRO, mappings={k: str(v).lower() for k, v in args.items()})
  return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def real():
  return expand()


@pytest.fixture(scope='module')
def sim():
  return expand(use_sim=True)


@pytest.fixture(scope='module')
def geometry():
  with open(GEOMETRY) as f:
    return yaml.safe_load(f)


def joints(robot):
  return {j.get('name'): j for j in robot.findall('joint')}


def origin(joint):
  o = joint.find('origin')
  xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
  rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
  return xyz + rpy


def close(a, b):
  return all(abs(x - y) < TOL for x, y in zip(a, b))


def test_tree_is_valid(real, sim):
  for robot in (real, sim):
    links = {l.get('name') for l in robot.findall('link')}
    children = set()
    for j in robot.findall('joint'):
      assert j.find('parent').get('link') in links, j.get('name')
      assert j.find('child').get('link') in links, j.get('name')
      children.add(j.find('child').get('link'))
    assert links - children == {'base_footprint'}, 'base_footprint must be the only root'


def test_real_and_sim_have_the_same_frames(real, sim):
  # Same links, same joint origins: TF is identical in sim and on the robot.
  assert {l.get('name') for l in real.findall('link')} == \
         {l.get('name') for l in sim.findall('link')}
  jr, js = joints(real), joints(sim)
  assert jr.keys() == js.keys()
  for name in jr:
    assert close(origin(jr[name]), origin(js[name])), f'{name} differs between real and sim'


def test_gazebo_tags_only_in_sim(real, sim):
  assert real.findall('gazebo') == []
  assert sim.findall('gazebo')


def test_wheel_joint_types(real, sim):
  wheels = ('left_wheel_joint', 'right_wheel_joint')
  real_with_js = joints(expand(driver_joint_states=True))
  for name in wheels:
    assert joints(real)[name].get('type') == 'fixed'
    assert joints(sim)[name].get('type') == 'continuous'
    assert real_with_js[name].get('type') == 'continuous'


def test_wheel_track_matches_geometry(sim, geometry):
  track = geometry['wheels']['track']
  if track == 'cad':
    pytest.skip('track taken from the CAD')
  j = joints(sim)
  assert abs(origin(j['left_wheel_joint'])[1] - track / 2) < TOL
  assert abs(origin(j['right_wheel_joint'])[1] + track / 2) < TOL

  plugin = next(p for g in sim.findall('gazebo') for p in g.findall('plugin')
                if p.get('filename') == 'libgazebo_ros_diff_drive.so')
  assert abs(float(plugin.find('wheel_separation').text) - track) < TOL
  radius = origin(j['left_wheel_joint'])[2]
  assert abs(float(plugin.find('wheel_diameter').text) - 2 * radius) < TOL


def test_lidar_matches_geometry(sim, geometry):
  spec = geometry['lidar_sensor']
  ray = sim.find("gazebo[@reference='laser']/sensor")
  assert float(ray.find('update_rate').text) == spec['rate']
  assert int(ray.find('ray/scan/horizontal/samples').text) == spec['samples']
  assert float(ray.find('ray/range/min').text) == spec['range_min']
  assert float(ray.find('ray/range/max').text) == spec['range_max']


def test_scan_plane_clears_the_body(real):
  # The simulated rays must not hit the robot's own chassis.
  base = next(l for l in real.findall('link') if l.get('name') == 'base_link')
  box = base.find('collision')
  top = float(box.find('origin').get('xyz').split()[2]) + \
      float(box.find('geometry/box').get('size').split()[2]) / 2
  scan_z = origin(joints(real)['lidar_joint'])[2]
  assert scan_z > top + 0.005, f'scan plane {scan_z:.3f} m vs body top {top:.3f} m'


def test_meshes_exist(real):
  prefix = 'package://hbot_description/'
  for mesh in real.iter('mesh'):
    path = mesh.get('filename')
    assert path.startswith(prefix), path
    assert os.path.isfile(os.path.join(PKG, path[len(prefix):])), path


def test_driver_wheels_match(geometry):
  # hbot_driver computes odometry from wheel_track and wheel_diameter; they
  # must be the wheels the description (and so Gazebo) uses. `cad` entries
  # aren't compared (the driver keeps its own value until one is measured).
  # Skipped when hbot_bringup isn't built.
  from ament_index_python.packages import (PackageNotFoundError,
                                           get_package_share_directory)
  try:
    share = get_package_share_directory('hbot_bringup')
  except PackageNotFoundError:
    pytest.skip('hbot_bringup not found')
  params = os.path.join(share, 'config', 'yahboom_driver_params.yaml')
  if not os.path.isfile(params):
    pytest.skip('yahboom_driver_params.yaml not installed')
  with open(params) as f:
    driver = yaml.safe_load(f)['hbot_driver_yahboom_node']['ros__parameters']
  track = geometry['wheels']['track']
  assert track == 'cad' or abs(driver['wheel_track'] - track) < TOL, (
    f"yahboom_driver_params.yaml wheel_track={driver['wheel_track']} but "
    f'config/hbot_geometry.yaml wheels.track={track}')
  radius = geometry['wheels']['radius']
  assert radius == 'cad' or abs(driver['wheel_diameter'] - 2 * radius) < TOL, (
    f"yahboom_driver_params.yaml wheel_diameter={driver['wheel_diameter']} but "
    f'config/hbot_geometry.yaml wheels.radius={radius} (diameter {2 * radius:g})')
