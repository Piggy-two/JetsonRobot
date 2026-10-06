#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Control Skill 的**运动规划逻辑（纯 Python，不依赖 ROS）**。

单独成文件的原因和 `safety_gate.py` 一样：这些是**安全依据**（"该走多远、多快、走多久"），
必须能离线证伪，不能靠"跑起来看着像对的"。

第一版刻意**只做开环、恒定机体速度、纯平移 / 纯旋转**：

    move_relative(x, y)  →  速度 = 单位方向 × 标称速度，时长 = 距离 / 标称速度
    rotate(angle)        →  角速度 = 符号 × 标称角速度，时长 = |angle| / 标称角速度

为什么不做「平移 + 旋转一次完成」：对全向底盘，**机体速度恒定 ≠ 世界系走直线** ——
同时旋转时终点位置会随朝向弯曲。要正确做"走到某相对位姿"，必须先解算
「转到目标朝向 → 平移 → 再转回」，而且**必须有里程计反馈**才能闭合。
开环硬凑出来的东西会"看起来动了、位置是错的"。所以第一版把这件事**明确不做**，
而不是做得似是而非。

⚠️ **开环的固有局限**（必须写清楚，别让上层误以为它闭环）：
  这只是"按时间发速度"。轮子打滑、地面摩擦、负载、电池电压都会让**实际**位移偏离。
  真正的闭环要么用 `/odom`（⚠️ 但 D-021 实测断线时 `/odom` 照发，**不能当存活判据**），
  要么用 LiDAR/视觉。那是后续的事，需要时再决策。
"""

from collections import namedtuple
from math import hypot

# 一次规划的结果：机体速度 + 该速度保持多久
Plan = namedtuple('Plan', 'vx vy wz duration')

# 小到不值得动 / 会造成除零的请求，直接当"已经在目标上"
EPS = 1e-3


class PlanError(Exception):
    """请求无法被安全地规划成一次运动。"""


def _check_finite(name, value):
    if value != value or value in (float('inf'), float('-inf')):
        raise PlanError(f'{name} 不是有限数（{value}）')


def plan_translate(dx, dy, speed, max_distance):
    """把「机体坐标系下平移 (dx, dy) 米」规划成一次恒定速度运动。

    :param speed: 标称线速度（m/s），必须是正值且**不应超过底盘限幅**
    :param max_distance: 单次允许的最大位移（m）—— 一道**本项目自己**的护栏，
                        不是靠 Motor Driver 的限幅兜底（那条管的是"速度"，不是"走多远"）
    :return: Plan，或 None 表示"无需运动"
    :raises PlanError: 请求不可接受
    """
    _check_finite('x', dx)
    _check_finite('y', dy)
    _check_finite('speed', speed)
    _check_finite('max_distance', max_distance)
    if speed <= 0:
        raise PlanError(f'标称速度必须为正，得到 {speed}')
    if max_distance <= 0:
        raise PlanError(f'max_distance 必须为正，得到 {max_distance}')

    dist = hypot(dx, dy)
    if dist < EPS:
        return None
    if dist > max_distance:
        raise PlanError(f'请求位移 {dist:.3f} m 超过单次上限 {max_distance:.3f} m')

    ux, uy = dx / dist, dy / dist
    return Plan(ux * speed, uy * speed, 0.0, dist / speed)


def plan_rotate(angle, rate, max_angle):
    """把「原地旋转 angle 弧度」规划成一次恒定角速度运动（逆时针为正）。"""
    _check_finite('angle', angle)
    _check_finite('rate', rate)
    _check_finite('max_angle', max_angle)
    if rate <= 0:
        raise PlanError(f'标称角速度必须为正，得到 {rate}')
    if max_angle <= 0:
        raise PlanError(f'max_angle 必须为正，得到 {max_angle}')

    if abs(angle) < EPS:
        return None
    if abs(angle) > max_angle:
        raise PlanError(f'请求转角 {angle:.3f} rad 超过单次上限 {max_angle:.3f} rad')

    sign = 1.0 if angle > 0 else -1.0
    return Plan(0.0, 0.0, sign * rate, abs(angle) / rate)


def within_chassis_limits(plan, max_vx, max_vy, max_wz):
    """再确认一次规划出来的速度没超底盘限幅。

    理论上前面的标称速度就该在限幅内，但那是**配置约定**；这里是**代码断言**。
    配错参数时宁可拒绝运动，也不要发出一个会被 Motor Driver 静默钳掉的速度 ——
    被钳掉意味着**实际走的距离和时间都对不上**，而调用方拿到的却还是 success。
    """
    if plan is None:
        return True, ''
    bad = []
    if abs(plan.vx) > max_vx + 1e-9:
        bad.append(f'vx={plan.vx:.3f} > {max_vx}')
    if abs(plan.vy) > max_vy + 1e-9:
        bad.append(f'vy={plan.vy:.3f} > {max_vy}')
    if abs(plan.wz) > max_wz + 1e-9:
        bad.append(f'wz={plan.wz:.3f} > {max_wz}')
    return (not bad), '；'.join(bad)
