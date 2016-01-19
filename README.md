# rqt_joint_trajectory_controller working on hydro

As `rqt_joint_trajectory_controller` is not supported in ROS Hydro, this is a little repo that contains it and also controller_manager_msgs (which contains some utils that the rqt tool needs) so it can be used in Hydro.

It's based on the indigo branches of both packages.

No guarantees of it working for everyone given.

Put this repo in your workspace, catkin_make it, source it and you can run it either:

    rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

And you'll get a single instance of the GUI to move around a group of joints.

Or:

    rosrun rqt_gui rqt_gui

And going to Plugins > Robot Tools > Joint trajectory controller you can add as many as you may fit in your screen.

(Note that you may need to do rosrun `rqt_gui rqt_gui --force-discover` once in order to see the plugin).

# joint_states_group_grabber.py

As a bonus `joint_states_group_grabber.py` is included which helps on getting the joint values for a group of the robot.

Use `-h` for help, `-i` for interactive mode. In interactive mode write exit to exit.

Example output:

````
$ ./joint_states_group_grabber.py -i
[INFO] [WallTime: 1453205262.563013] [811.565000] Node initialized. Ready to grab joint states.
Interactive mode! Write a group name (or short group name) and it's values will be printed. (Write exit to exit).
['all_joints', 'left_arm', 'right_arm', 'both_arms', 'left_arm_torso', 'right_arm_torso', 'both_arms_torso', 'torso', 'head', 'right_hand', 'left_hand', 'right_hand_all', 'left_hand_all']
['a', 'la', 'ra', 'ba', 'lat', 'rat', 'bat', 't', 'h', 'rh', 'lh', 'rha', 'lha']
> la
Name =  Joint Value
=================
arm_left_1_joint = 8.92178505918e-05
arm_left_2_joint = -0.000769733377565
arm_left_3_joint = -1.05322722881e-05
arm_left_4_joint = 0.000489904335971
arm_left_5_joint = 2.27394518468e-05
arm_left_6_joint = -0.000106191031183
arm_left_7_joint = -2.98292162677e-05
left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']
[  8.92178505918e-05, -0.000769733377565, -1.05322722881e-05, 0.000489904335971, 2.27394518468e-05, -0.000106191031183, -2.98292162677e-05 ]
````


