figleaf
=======

Privacy tools for robotics.

**Dependencies**

* ROS Groovy
* ROS workspaces located at `${HOME}/catkin_ws` and `${HOME}/rosbuild_ws`
* [rviz_camera_publisher](https://github.com/jstnhuang/rviz_camera_publisher) (depends on catkin version of view_controller_msgs)
  * [view_controller_msgs](https://github.com/ros-visualization/view_controller_msgs) (use hydro-devel to get the catkin version)
* [rviz_objdetect_caller](https://github.com/jstnhuang/rviz_objdetect_caller)
  * geometry_msgs
  * object_manipulation_msgs
  * pr2_interactive_object_detection
  * sensor_msgs
  * [scikit-image](http://scikit-image.org/download.html)
* Modified version of rviz (see instructions below)

**For UW CSE users**

For help setting up ROS, see https://sites.google.com/site/humancenteredrobotics/internal/set-up-to-work-on-pbd-in-014 .

**Install**

1. Install view_controller_msgs and rviz_camera_publisher to the catkin workspace and compile.
  ```
  cd ~/catkin_ws/src
  git clone git@github.com:ros-visualization/view_controller_msgs.git -b hydro-devel
  git clone git@github.com:jstnhuang/rviz_camera_publisher.git
  cd ~/catkin_ws
  catkin_make
  ```
  
2. Install rviz_objdetect_caller to the rosbuild workspace on the workstation.
  ```
  cd ~/rosbuild_ws  
  git clone git@github.com:jstnhuang/rviz_objdetect_caller.git
  rosmake
  ```

3. Install scikit-image:
   ```sudo pip install -U scikit-image```

4. Clone this repo to your home folder, ~/figleaf. Run scripts/install.py to create symbolic links in your catkin and rosbuild workspaces. Run `catkin_make` in your catkin workspace.
5. Make a modified version of rviz in your catkin workspace:
   ```
   cd ~/catkin_ws/src
   git clone git@github.com:ros-visualization/rviz.git -b groovy-devel
   ```
   The modification disables the "Kinect Stream" display. Instead, it uses the code in the Kinect Stream display that computes the point cloud shadow, and publishes the point cloud and shadow point cloud to a topic called /kinect_mld (multi-layer display).
   When the experiment is launched, it launches a node which sends /kinect_mld through a self-filtering nodes, which eliminates points that just represent the robot. The filtered point cloud is published to /kinect_mld_filtered.

5. Optional: make the tabletop segmenter work on self-filtered point clouds. Solution for now: on the robot, run
   ```sudo vim /opt/ros/groovy/stacks/pr2_object_manipulation/applications/pr2_interactive_object_detection/launch/pr2_interactive_object_detection_robot.launch```
   
   Change tabletop_segmentation_points_inputs from `$(arg kinect_camera_name)/depth_registered/points` to `/kinect_mld_filtered`.

   Before:
   ```
   <!-- Launch default tabletop detector -->
    <include unless="$(arg cvfh)" file="$(find tabletop_object_detector)/launch/tabletop_complete.launch">
      <arg name="tabletop_segmentation_points_input" value="$(arg kinect_camera_name)/depth_registered/points"/>
      <arg name="flatten_table" value="$(arg flatten_table)"/>  
      <arg name="model_set" value="$(arg model_set)"/> 
    </include>

    <!-- Launch CVFH -->
    <include if="$(arg cvfh)" file="$(find tabletop_vfh_cluster_detector)/launch/tabletop_complete.launch">
      <arg name="tabletop_segmentation_points_input" value="$(arg kinect_camera_name)/depth_registered/points"/>
      <arg name="flatten_table" value="$(arg flatten_table)"/>  
    </include>
    ```
    After:
    ```
    <include unless="$(arg cvfh)" file="$(find tabletop_object_detector)/launch/tabletop_complete.launch">
      <!--/head_mount_kinect/rgb/object_modeling_points_filtered-->
      <arg name="tabletop_segmentation_points_input" value="/kinect_mld_filtered"/>
      <arg name="flatten_table" value="$(arg flatten_table)"/>
      <arg name="model_set" value="$(arg model_set)"/>
    </include>
    
    <include if="$(arg cvfh)" file="$(find tabletop_vfh_cluster_detector)/launch/tabletop_complete.launch">
      <arg name="tabletop_segmentation_points_input" value="/kinect_mld_filtered"/>
      <arg name="flatten_table" value="$(arg flatten_table)"/>
    </include>
    ```

**Running the experiment**

In 014, shut off the right light switch.

On the robot, run interactive manipulation: `roslaunch pr2_interactive_manipulation pr2_interactive_manipulation_robot.launch`

On the workstation, run `scripts/experiment.sh 123 clean`, where 123 is the user ID, and "clean" is the filter to use. The filters are:
* *clean*: No filters at all
* *mid*: A superpixel and hue-inverted image + point cloud of only segmented objects
* *box*: A filter where segmented objects are shown as boxes.

The experiment script will save rosbag recordings to ~/experiment_data. Modify the script if you want to change where the data goes.

To change what topics get logged or what nodes get run, run `rosed figleaf_2d experiment.launch` to change the launch file.

After, you can get the time taken for each condition using `python scripts/process_bag.py ~/experiment_data/123_clean.bag`.
