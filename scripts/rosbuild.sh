#!/bin/bash
cd ${HOME}/rosbuild_ws
rosmake `ls -d */ | cut -f1 -d'/'`
