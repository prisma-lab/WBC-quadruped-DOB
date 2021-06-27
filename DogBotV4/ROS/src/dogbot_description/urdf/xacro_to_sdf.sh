#!/usr/bin/env bash
echo "Cleaning sdf and urdf"
rm -rf dogbot.sdf dogbot.urdf
rosrun xacro xacro.py dogbot.xacro > dogbot.urdf
gz sdf -p dogbot.urdf > dogbot.sdf
echo "Generated SDF"
