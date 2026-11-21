"""URDF-to-KDL chain, installation/TCP transforms and bounded native IK."""
import math
import time
import xml.etree.ElementTree as ET

import PyKDL as kdl


def xyz(text):
    return [float(v) for v in text.split()]


def origin(element):
    if element is None:
        return kdl.Frame.Identity()
    p = xyz(element.get('xyz', '0 0 0'))
    a = xyz(element.get('rpy', '0 0 0'))
    return kdl.Frame(kdl.Rotation.RPY(*a), kdl.Vector(*p))


def configured_frame(value):
    q, p = value['rotation'], value['translation']
    return kdl.Frame(kdl.Rotation.Quaternion(q['x'], q['y'], q['z'], q['w']),
                     kdl.Vector(p['x'], p['y'], p['z']))


def array(values):
    result = kdl.JntArray(len(values))
    for i, value in enumerate(values):
        result[i] = value
    return result


class Kinematics:
    """Require Gazebo URDF, intersect configured limits with model limits."""

    def __init__(self, xml, config):
        self.config = config
        robot = ET.fromstring(xml)
        # This execution backend is intentionally simulation-only.
        if not any('gz_ros2_control' in e.get('filename', '') for e in robot.iter('plugin')):
            raise ValueError('gazebo backend requires a gz_ros2_control simulation URDF')
        by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
        selected = []
        link = config['frames']['flange']
        visited = set()
        while link != config['frames']['base']:
            if link in visited or link not in by_child:
                raise ValueError('base-to-flange chain is missing or cyclic')
            visited.add(link)
            j = by_child[link]
            selected.append(j)
            link = j.find('parent').get('link')
        self.chain = kdl.Chain()
        self.names, self.lower, self.upper = [], [], []
        cfg_joints = {j['name']: j for j in config['joints']}
        for j in reversed(selected):
            name, kind = j.get('name'), j.get('type')
            frame = origin(j.find('origin'))
            if kind == 'fixed':
                joint = kdl.Joint(name, kdl.Joint.Fixed)
            elif kind in ('revolute', 'continuous'):
                if name not in cfg_joints:
                    raise ValueError('configured joints do not match URDF chain')
                axis_e = j.find('axis')
                axis = kdl.Vector(*xyz(axis_e.get('xyz', '1 0 0') if axis_e is not None
                                       else '1 0 0'))
                if abs(axis.Norm()-1.) > 1e-6:
                    raise ValueError('joint axis must be normalized')
                joint = kdl.Joint(name, frame.p, frame.M * axis, kdl.Joint.RotAxis)
                c = cfg_joints[name]
                lo, hi = c['lower_rad'], c['upper_rad']
                limits = j.find('limit')
                if limits is None:
                    raise ValueError('joint limit missing from URDF')
                if kind == 'revolute':
                    lo = max(lo, float(limits.get('lower')))
                    hi = min(hi, float(limits.get('upper')))
                if lo >= hi or c['max_velocity_rad_s'] > float(limits.get('velocity')):
                    raise ValueError('configuration exceeds model joint limits')
                self.names.append(name)
                self.lower.append(lo)
                self.upper.append(hi)
            else:
                raise ValueError('only fixed/revolute/continuous joints supported')
            self.chain.addSegment(kdl.Segment(j.find('child').get('link'), joint, frame))
        if self.names != [j['name'] for j in config['joints']]:
            raise ValueError('config joint order must equal canonical URDF chain order')
        self.base = configured_frame(config['frames']['reference_to_base'])
        self.tool = configured_frame(config['frames']['flange_to_tcp'])
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.vel_solver = kdl.ChainIkSolverVel_pinv(self.chain, 1e-7, 100)
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(
            self.chain, array(self.lower), array(self.upper), self.fk_solver,
            self.vel_solver, config['ik']['max_iterations'], 1e-7)

    def fk(self, values):
        frame = kdl.Frame()
        if self.fk_solver.JntToCart(array(values), frame) < 0:
            raise ValueError('FK failed')
        return self.base * frame * self.tool

    def twist(self, q, dq):
        jac = kdl.Jacobian(len(q))
        if self.jac_solver.JntToJac(array(q), jac) < 0:
            raise ValueError('Jacobian failed')
        v = [sum(jac[row, col]*dq[col] for col in range(len(q))) for row in range(6)]
        flange = kdl.Frame()
        self.fk_solver.JntToCart(array(q), flange)
        omega = kdl.Vector(*v[3:])
        linear = kdl.Vector(*v[:3]) + omega * (flange.M * self.tool.p)
        return self.base.M * linear, self.base.M * omega

    def solve(self, pose, seed):
        start = time.monotonic_ns()
        target = self.base.Inverse() * pose * self.tool.Inverse()
        result = kdl.JntArray(len(seed))
        rc = self.ik_solver.CartToJnt(array(seed), target, result)
        elapsed = time.monotonic_ns()-start
        if elapsed > self.config['timing']['ik_budget_ns']:
            raise ValueError('IK deadline exceeded')
        if rc < 0:
            raise ValueError(f'IK failed ({rc})')
        q = [result[i] for i in range(len(seed))]
        if any(not math.isfinite(v) or not lo <= v <= hi
               for v, lo, hi in zip(q, self.lower, self.upper)):
            raise ValueError('IK joint range violation')
        residual = kdl.diff(self.fk(q), pose)
        if (residual.vel.Norm() > self.config['ik']['position_tolerance_m'] or
                residual.rot.Norm() > self.config['ik']['orientation_tolerance_rad']):
            raise ValueError('IK residual exceeds tolerance')
        return q, elapsed
