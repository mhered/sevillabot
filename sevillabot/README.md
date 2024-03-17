## Robot Package

This package, originally based on GitHub template:  https://github.com/joshnewans/my_bot , was included into https://github.com/mhered/manolobot and customized for the **manolobot** robot configuration into the ROS2 package `manolobot_uno`.

It was then copied to the `sevillabot` project and further edited to reflect the configuration of the **sevillabot** robot. This folder is symlinked to `~/dev_ws/src/`, and built into the ROS2 package `sevillabot`.

Each directory has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has directories present for CMake to find). Example files can be removed if required.

**Note: To remove the directories adjust  `CMakeLists.txt`  accordingly **