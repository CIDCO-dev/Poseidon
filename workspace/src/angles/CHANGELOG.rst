^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package angles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.13 (2020-03-11)
-------------------
* Update the angle normalization function to a simpler implementation (`#19 <https://github.com/ros/angles/issues/19>`_)
  * Update the angle normalization function for a simpler alternative
  * Simplify 2*pi angle wrapping.
  * Simplify/fasten the C++ implementation of angle normalization (removes one fmod call)
* Bump CMake version to avoid CMP0048 warning (`#20 <https://github.com/ros/angles/issues/20>`_)
* Contributors: Alexis Paques, Shane Loretz

1.9.12 (2020-01-08)
-------------------
* Added support for "large limits" (`#16 <https://github.com/ros/angles/issues/16>`_)
* Small documentation updates.
* Contributors: Franco Fusco, Tully Foote

1.9.11 (2017-04-14)
-------------------
* Add a python implementation of angles
* Do not use catkin_add_gtest if CATKIN_ENABLE_TESTING
* Contributors: David V. Lu, David V. Lu!!, Ryohei Ueda

1.9.10 (2014-12-29)
-------------------
* Export architecture_independent flag in package.xml
* Simply and improve performance of shortest_angular_distance(). adding two unit test cases
* check for CATKIN_ENABLE_TESTING
* Contributors: Derek King, Lukas Bulwahn, Scott K Logan, Tully Foote

1.9.9 (2013-03-23)
------------------
* catkin as a buildtool dependency
* Contributors: Tully Foote

1.9.8 (2012-12-05)
------------------
* Removed 'copyright' tag from package.xml
* Contributors: William Woodall

1.9.7 (2012-10-02 21:23)
------------------------
* fix typo
* Contributors: Vincent Rabaud

1.9.6 (2012-10-02 15:39)
------------------------
* comply to the new catkin API
* Contributors: Vincent Rabaud

1.9.5 (2012-09-16 18:11)
------------------------
* fixes to build
* Contributors: Ioan Sucan

1.9.4 (2012-09-16 01:24)
------------------------
* rename the include folder to angles as it should be
* Contributors: Vincent Rabaud

1.9.3 (2012-09-03 00:55)
------------------------
* fix relative path
* Contributors: Ioan Sucan

1.9.2 (2012-09-03 00:32)
------------------------

1.9.1 (2012-07-24)
------------------
* add proper manifest
* Contributors: Ioan Sucan

1.9.0 (2012-07-23)
------------------
* fix the test for the new headers
* fix the guard
* package builds with catkin
* remove useless header
* copying from geometry/
* Contributors: Ioan Sucan, Vincent Rabaud
