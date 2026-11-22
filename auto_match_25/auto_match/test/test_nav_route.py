#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_route 纯逻辑单测（无需 ROS）。

运行：
    python3 test/test_nav_route.py
或
    python3 -m unittest discover -s test
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nodes"))

from nav_route import distance, nearest, distance_matrix, greedy_order


class TestNavRoute(unittest.TestCase):
    def test_distance(self):
        self.assertAlmostEqual(distance((0, 0), (3, 4)), 5.0)

    def test_nearest(self):
        cands = {"a": (1, 0), "b": (10, 0)}
        name, d = nearest((0, 0), cands)
        self.assertEqual(name, "a")
        self.assertAlmostEqual(d, 1.0)

    def test_nearest_empty(self):
        name, d = nearest((0, 0), {})
        self.assertIsNone(name)
        self.assertEqual(d, float("inf"))

    def test_distance_matrix(self):
        m = distance_matrix({"o": (0, 0)}, {"a": (3, 4)})
        self.assertAlmostEqual(m["o"]["a"], 5.0)

    def test_greedy_single(self):
        targets = {"S1": (1, 0), "S2": (5, 0), "S3": (10, 0)}
        order = greedy_order((0, 0), targets)
        self.assertEqual(order, ["S1", "S2", "S3"])

    def test_greedy_no_consecutive_repeat(self):
        targets = {"S1": (1, 0), "S2": (5, 0)}
        order = greedy_order((0, 0), targets, {"S1": 2, "S2": 1})
        # 先从近的 S1，再一次不能连续同一处，转 S2，最后回来 S1
        self.assertEqual(order, ["S1", "S2", "S1"])

    def test_greedy_counts_exhausted(self):
        targets = {"S1": (1, 0)}
        order = greedy_order((0, 0), targets, {"S1": 3})
        self.assertEqual(order, ["S1", "S1", "S1"])


if __name__ == "__main__":
    unittest.main()
