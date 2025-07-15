#!/bin/bash

source /opt/ros/jazzy/local_setup.bash

# Create user ros, and allow it to install stuff. 
adduser --disabled-password --gecos "docker user" ubuntu
echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

# clean up apt
#     
su ubuntu

# clean up apt
#     
su ubuntu

cd /home/ubuntu/
git clone --recurse-submodules https://github.com/johnny555/ros-quickstart-rrrw.git
cd /home/ubuntu/ros-quickstart-rrrw

# install build deps and build
sudo apt update
rosdep install --from-path src --ignore-src -y -r --rosdistro jazzy
colcon build --symlink-install --merge-install

sudo rm -rf /var/lib/apt/lists/*

# Make it so that sourcing happens automatically
echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc
echo "source /workspace/install/setup.bash" >> /home/ubuntu/.bashrc

# Suppress deprecated setuptools warning
echo "PYTHONWARNINGS=\"ignore:setup.py install is deprecated::setuptools.command.install,ignore:easy_install command is deprecated::setuptools.command.easy_install\"; export PYTHONWARNINGS" >> /home/ubuntu/.bashrc

# Add GAZEBO path so we can easily include models. Must use merge install. 
# Sholud be able to use package:// syntax now. 
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/install/share/:/opt/ros/jazzy/share/" >> /home/ubuntu/.bashrc

echo "export SDF_PATH=$SDF_PATH:/workspace/install/share/:/opt/ros/jazzy/share/" >> /home/ubuntu/.bashrc