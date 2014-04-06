live-cd
=======

Ubuntu/ROS custum live CD creator


1. Edit [script](https://github.com/tork-a/live-cd/blob/master/00-create-cd.sh) and commit changes
2. Wait [jenkins](http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/live-cd/) for create iso image
3. Download latest iso image from [artifacts](http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/live-cd/lastSuccessfulBuild/artifact/)
4. Burn iso to DVD or USB ( see https://help.ubuntu.com/community/Installation/FromUSBStick for mor instructions )
