gaia
====


export GAZEBO_PLUGIN_PATH=~/gaia/gazebo_plugin/build:${GAZEBO_PLUGIN_PATH}

export GAZEBO_MODEL_PATH=~/gaia/gazebo_plugin/models:${GAZEBO_MODEL_PATH}

export GAZEBO_RESOURCE_PATH=~/gaia/gazebo_plugin/models:${GAZEBO_RESOURCE_PATH}

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc