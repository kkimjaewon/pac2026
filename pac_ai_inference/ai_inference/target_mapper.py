import numpy as np


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def pixel_to_camera_link(u, v, depth_m, fx, fy, cx, cy):
    """영상(optical: X 오른쪽, Y 아래, Z 앞) -> Gazebo 센서 링크(x 앞, y 왼쪽, z 위)"""
    x_opt = (u - cx) * depth_m / fx
    y_opt = (v - cy) * depth_m / fy
    return np.array([depth_m, -x_opt, -y_opt])


def camera_link_to_map(p_link, translation, quat_xyzw):
    return quat_to_matrix(*quat_xyzw) @ np.asarray(p_link) + np.asarray(translation)
