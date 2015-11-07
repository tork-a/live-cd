live-cd/USB: Ubuntu/ROS custom live CD/USB creator
========================================================

Description
------------------------------------------------

This repository hosts scripts to create Ubuntu `.iso` images where [ROS](http://ros.org/) is pre-installed.

1. What are the generated .iso files?

 * The generated .iso works as the normal Ubuntu .iso image. ROS is already installed too. Versions depend on which .iso you download from artifacts page.
 * It keeps DEB files that were used during .iso generation 

2. So, what can I do with these generated .iso files for example?

 * (general Ubuntu .iso feature) You can run Ubuntu on your computer from the CD/thumbdrive, without installing it. Also you can install Ubuntu on your computer.
 * In the above scenario, ROS is already installed.
 * You can use a thumbdrive made with these .iso images as as the local DEB repository. So for instance, if your computer is disconnected from internet but still want to install some ROS packages from DEB (ie. by apt), this works.

How to create new .iso images
------------------------------------------------

1. Edit [script](https://github.com/tork-a/live-cd/blob/master/00-create-cd.sh) and commit changes
2. Run commands
```
sudo apt-get install -y uck
sudo rm -fr ~/tmp/remaster-*
./00-create-cd.sh  --rosdistro indigo
./00-create-cd.sh  --local-repo --rosdistro indigo # for installing local repository
sudo ./00-check-cd.sh tork-ubuntu-ros-14.04-amd64-*.iso # check generated iso file
```
3. Burn iso to DVD or USB ( see https://help.ubuntu.com/community/Installation/FromUSBStick for more instructions )

Using custom live CD as local DEB repository
------------------------------------------------

1. Assuming that your USB drive is located at /media/USB\ DISK, using `ls -al /media/USB\ DISK`
2. Setup `sources.list` as follows, make sure that you have comment out ROS repository
```
sudo su -c 'echo "deb file:///media/USB%20DISK/repository binary/" > /etc/apt/sources.list.d/tork-usb.list'
sudo su -c 'sed -i "s/deb/# deb/" /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
```
3. Now, you can `apt-get install` without Internet connection
4. Do not forget to comment in ROS repository after you finish working
```
sudo su -c 'sed -i "s/# deb/deb/" /etc/apt/sources.list.d/ros-latest.list'
```
