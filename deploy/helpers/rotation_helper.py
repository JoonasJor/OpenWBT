import numpy as np
from scipy.spatial.transform import Rotation as R

def quat_inv_np(q: np.ndarray) -> np.ndarray:
    """
    inverse of quaternions
    Accepts numpy.ndarray as input.
    """
    w = -1 * q[..., -1:]
    xyz = q[..., :3]
    return np.hstack([xyz, w])


def broadcast_quat_apply_np(q: np.ndarray, vec3: np.ndarray) -> np.ndarray:
    t = 2 * np.cross(q[..., :3], vec3, axis=-1)
    xyz = vec3 + q[..., 3, None] * t + np.cross(q[..., :3], t, axis=-1)
    return xyz


def broadcast_quat_multiply_np(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Multiply 2 quaternions. p.shape == q.shape
    """

    w: np.ndarray = p[..., 3:4] * q[..., 3:4] - np.sum(p[..., :3] * q[..., :3], axis=-1, keepdims=True)
    xyz: np.ndarray = (
            p[..., 3, None] * q[..., :3] + q[..., 3, None] * p[..., :3] + np.cross(p[..., :3], q[..., :3], axis=-1))

    return np.concatenate([xyz, w], axis=-1)


def facing_to_world(pos_root, quat_root, point_facing):
    """
    将facing坐标系下的点转换到世界坐标系。

    :param pos_root: 机器人root在世界坐标系下的位置，形如 [x, y, z]
    :param quat_root: 机器人root在世界坐标系下的四元数，形如 [x, y, z, w]
    :param point_facing: facing坐标系下的点，形如 [x, y, z]
    :return: 点在世界坐标系下的坐标，形如 [x, y, z]
    """
    # 将四元数转为旋转矩阵
    R_root_to_world = R.from_quat(quat_root).as_matrix()

    # 提取 root 坐标系的 x 和 y 轴方向
    x_axis_world = R_root_to_world[:, 0]  # root 坐标系的 x 轴在世界坐标系下的方向
    y_axis_world = R_root_to_world[:, 1]  # root 坐标系的 y 轴在世界坐标系下的方向

    # 计算地面投影（将 z 分量置为 0）
    x_axis_ground = np.array([x_axis_world[0], x_axis_world[1], 0.0])
    y_axis_ground = np.array([y_axis_world[0], y_axis_world[1], 0.0])

    # 归一化地面投影向量
    x_axis_facing = x_axis_ground / np.linalg.norm(x_axis_ground)
    z_axis_facing = np.array([0.0, 0.0, 1.0])  # facing 坐标系的 z 轴与世界坐标系的 z 轴一致
    y_axis_facing = np.cross(z_axis_facing, x_axis_facing)  # 保证正交性

    # 构造 facing 到 root 的旋转矩阵
    R_facing_to_root = np.vstack([x_axis_facing, y_axis_facing, z_axis_facing]).T

    # facing 坐标系下的点转到 root 坐标系
    point_root = R_facing_to_root @ point_facing

    # root 坐标系下的点转到世界坐标系
    point_world = R_root_to_world @ point_root + pos_root

    return point_world


def get_euler_xyz(q):
    # 提取四元数的分量
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]

    # roll (x轴旋转)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = qw * qw - qx * qx - qy * qy + qz * qz
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y轴旋转)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.where(
        np.abs(sinp) >= 1, np.sign(sinp) * (np.pi / 2.0), np.arcsin(sinp))

    # yaw (z轴旋转)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = qw * qw + qx * qx - qy * qy - qz * qz
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def transform_imu_data(waist_yaw, waist_yaw_omega, imu_quat, imu_omega):
    RzWaist = R.from_euler("z", waist_yaw).as_matrix()
    R_torso = R.from_quat([imu_quat[1], imu_quat[2], imu_quat[3], imu_quat[0]]).as_matrix()
    R_pelvis = np.dot(R_torso, RzWaist.T)
    w = np.dot(RzWaist, imu_omega[0]) - np.array([0, 0, waist_yaw_omega])
    return R.from_matrix(R_pelvis).as_quat()[[3, 0, 1, 2]], w

