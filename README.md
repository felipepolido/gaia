gaia
====


sudo rm -r gaia/
git clone https://github.com/felipepolido/gaia
cd gaia/
source ~/.bashrc 
source /opt/ros/groovy/setup.sh
cd src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash 
cd src/
catkin_create_pkg gaia std_msgs rospy roscpp
roscd gaia
subl ../../README.md &







export GAZEBO_PLUGIN_PATH=~/gaia/gazebo_plugin/build:${GAZEBO_PLUGIN_PATH}

export GAZEBO_MODEL_PATH=~/gaia/gazebo_plugin/models:${GAZEBO_MODEL_PATH}

export GAZEBO_RESOURCE_PATH=~/gaia/gazebo_plugin/models:${GAZEBO_RESOURCE_PATH}

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc