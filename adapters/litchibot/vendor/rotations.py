"""Strict, scalar-first quaternion and ordered rotation math (radians)."""
from __future__ import annotations

import math
import numpy as np


def unit(value, size=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError(f"Expected a finite {size}-vector")
    length = float(np.linalg.norm(a))
    if length < 1e-10:
        raise ValueError("Zero quaternion/axis is not a measurement")
    return a / length


def matrix(quaternion):
    w, x, y, z = unit(quaternion, 4)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def quaternion(rotation):
    # Eigen formulation remains well conditioned at pi and is q/-q invariant.
    r = np.asarray(rotation, dtype=float)
    k = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)],
    ]) / 3
    _, v = np.linalg.eigh(k)
    q = v[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def log(rotation):
    r = np.asarray(rotation, dtype=float)
    v = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]])
    sine = .5*float(np.linalg.norm(v))
    cosine = float(np.clip((np.trace(r)-1)*.5, -1, 1))
    if cosine > 0 and sine < 1e-8:
        return .5*v
    if sine > 1e-6:
        return v*(math.atan2(sine,cosine)/(2*sine))
    # The skew part vanishes at pi; recover the axis without division by zero.
    q = quaternion(r)
    n = float(np.linalg.norm(q[1:]))
    return 2*q[1:] if n < 1e-10 else q[1:]*(2*math.atan2(n,q[0])/n)


def rotation(axis, angle):
    x, y, z = unit(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k@k)


def ordered(axes, angles):
    result = np.eye(3)
    for axis, angle in zip(axes, angles):
        result = result @ rotation(axis, float(angle))
    return result


def jacobian(axes, angles):
    prefix = np.eye(3)
    columns = []
    for axis, angle in zip(axes, angles):
        columns.append(prefix @ unit(axis))
        prefix = prefix @ rotation(axis, float(angle))
    return np.column_stack(columns)


def principal_twist(observed, axis):
    q = quaternion(observed)
    p = float(q[1:] @ unit(axis))
    if math.hypot(q[0], p) < 1e-8:
        raise ValueError("Unobservable axial rotation")
    theta = 2*math.atan2(p, q[0])
    return (theta + math.pi) % (2*math.pi) - math.pi
