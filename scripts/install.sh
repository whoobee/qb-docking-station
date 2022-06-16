
#make sure everything is up to date
###################################
sudo apt update
sudo apt upgrade

#git install
############
sudo apt-get install git -y

#python 2.7 and 3 install
sudo apt-get install python python-pip python3 python3-pip -y

#ros install and configure
##########################
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
echo "source ~/qb-docking-station/ros_ws/devel/setup.bash" >> ~/.bashrc
#echo "export QB_IP_ADDR=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
#echo "export ROS_MASTER_URI=http://${QB_IP_ADDR}:11311" >> ~/.bashrc
#echo "export ROS_HOSTNAME=${QB_IP_ADDR}" >> ~/.bashrc
#echo "export ROS_IP=${QB_IP_ADDR}" >> ~/.bashrc

#install qB-docking-station ROS dependecies
###########################################
#none so far

#clone qB and setup
########################
cd ~/
git clone https://github.com/whoobee/qb-docking-station.git
cd ~/qb-docking-station/ros_ws
catkin_make clean

#make qb-docking-station
########################
catkin_make

#install dynamixel library
##########################
pip3 install dynamixel-sdk

#install GPIO control
#####################
pip3 install RPi.GPIO
sudo apt install rpi.gpio-common
sudo adduser "${USER}" dialout

#install i2c tools
##################
sudo apt install -y i2c-tools
sudo pip3 install smbus

#setup access to i2c driver in user non-root
############################################
sudo groupadd i2c
sudo chown :i2c /dev/i2c-1
sudo chmod g+rw /dev/i2c-1
sudo usermod -aG i2c $USER
#maybe we need to make an udev rule:> # echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules

#enable power button listener script
####################################
#enable shutdown for all users
sudo chmod u+s /sbin/shutdown
