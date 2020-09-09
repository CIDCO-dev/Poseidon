#!/usr/bin/env python

import unittest
import rostest

class InitialTest(unittest.TestCase):

    def test_whatever(self):
        pass

if __name__ == "__main__":
    rostest.rosrun('intial test', 'initial_test', InitialTest)