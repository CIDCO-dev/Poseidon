#!/usr/bin/env python

import unittest

try:
    import pytest
except ImportError:
    pytest = None

if pytest:
    pytest.importorskip("rostest", reason="ROS Python modules are not available")
import rostest

class InitialTest(unittest.TestCase):

    def test_whatever(self):
        pass

if __name__ == "__main__":
    rostest.rosrun('intial test', 'initial_test', InitialTest)
