live-cd
=======

Ubuntu/ROS custum live CD creator


1. Edit [script](https://github.com/tork-a/live-cd/blob/master/00-create-cd.sh) and commit changes
2. Wait [jenkins](http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/live-cd/) for create iso image
3. Download latest iso image from [artifacts](http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/live-cd/lastSuccessfulBuild/artifact/)
4. Burn iso to DVD or USB ( see https://help.ubuntu.com/community/Installation/FromUSBStick for more instructions )


Using custom live CD as local repository

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
