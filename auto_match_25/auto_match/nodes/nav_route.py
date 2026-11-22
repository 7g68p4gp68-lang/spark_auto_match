#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航路径规划的纯逻辑（无 ROS，可单测）。

职责：
- 两点距离 / 距离矩阵
- 最近点选择
- 贪心顺序（近似 TSP，用于"抓完一个去最近的下一个分拣点"）

坐标一律是 map 坐标系下的 (x, y) 元组。
"""

import math


def distance(a, b):
    """两点欧氏距离。a/b 为 (x, y)。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest(origin, candidates):
    """在 candidates 里找离 origin 最近的点。

    :param origin: (x, y)
    :param candidates: {name: (x, y)} 或 [(name, (x, y)), ...]
    :return: (name, dist)；没有候选返回 (None, inf)
    """
    items = candidates.items() if isinstance(candidates, dict) else candidates
    best = None
    best_d = float("inf")
    for name, xy in items:
        if xy is None:
            continue
        d = distance(origin, xy)
        if d < best_d:
            best_d = d
            best = name
    return best, best_d


def distance_matrix(origins, targets):
    """距离矩阵。

    :param origins: {name: (x, y)}
    :param targets: {name: (x, y)}
    :return: {origin_name: {target_name: dist}}
    """
    matrix = {}
    for o_name, o_xy in origins.items():
        if o_xy is None:
            continue
        row = {}
        for t_name, t_xy in targets.items():
            if t_xy is None:
                continue
            row[t_name] = distance(o_xy, t_xy)
        matrix[o_name] = row
    return matrix


def greedy_order(origin, targets, counts=None):
    """从 origin 出发，贪心走完 targets（每个点可访问多次）。

    :param origin: (x, y) 起始坐标
    :param targets: {name: (x, y)}
    :param counts: {name: 需要访问次数}；None 表示每个点一次
    :return: [name, ...] 访问顺序（可能含重复）
    """
    remaining = dict(counts) if counts else {name: 1 for name in targets}
    remaining = {name: int(n) for name, n in remaining.items() if name in targets}
    current = origin
    last = None
    order = []
    while any(n > 0 for n in remaining.values()):
        cand = {name: targets[name] for name, n in remaining.items()
                if n > 0 and targets.get(name) is not None}
        if not cand:
            break
        # 按距离排序；最近的就是上一个点且还有别的可选时，换下一个，避免原地连续重复
        ranked = sorted(cand.items(), key=lambda kv: distance(current, kv[1]))
        chosen = ranked[0][0]
        if chosen == last and len(ranked) > 1:
            chosen = ranked[1][0]
        order.append(chosen)
        remaining[chosen] -= 1
        current = targets[chosen]
        last = chosen
    return order
