# avc_2018
Custom ROS package for the 2018 AVC Logistics Class competition

If you do not already have a catkin workspace set up, run the following

`mkdir -p ~/catkin_ws/src`
`cd ~/catkin_ws/src`

Once your in the src folder for your workspace, clone the repository

`git clone https://github.com/SDSU-Robotics/avc_2018.git`

You can then create a branch for yourself and create pull requests once your code is ready.

To switch to a different branch, run:

`git checkout BRANCH-NAME`

To create a branch, run

`git checkout -b BRANCH-NAME`

Make sure you are branching from the branch you want to be branching from! Checkout master first before creating your branch (unless you actually want to branch off another branch).

Once you're in whatever branch you want to be in run

`catkin_make`

You only have to do this when new files are introduced (unless you need to compile C++ code). If you opened a terminal before running `catkin_make`, you'll have to source your workspace manually (new terminals should source it automatically now).

`source ~/.bashrc`

bashrc should have a command to source your workspace in there. If that doesn't work, you can do it manually:

`source ~/catkin_ws/devel/setup.bash`
