#!/bin/bash
if [ -d "$HOME/catkin_ws/src/figleaf_2d" ]; then
    echo "figleaf appears to be installed."
else
    echo "installing figleaf..."
    ln -s ${HOME}/figleaf/catkin/figleaf_2d ${HOME}/catkin_ws/src/figleaf_2d
    echo "done"
fi

if [ -d "$HOME/rosbuild_ws/pr2_pbd" ]; then
    echo "pr2_pbd appears to be installed."
else
    echo "You need install pr2_pbd. Try forking https://github.com/PR2/pr2_pbd, and cloning it into your rosbuild workspace."
fi
