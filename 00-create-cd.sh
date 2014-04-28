#!/bin/bash
set -x
set -e

if [ $# -gt 0 ]; then ## if we have arguments, check functions
    DEBUG=TRUE
fi

# init stuff
if [ ! ${DEBUG} ]; then
    rm -fr ~/tmp/remaster-new-files/
    sudo uck-remaster-clean
    if [ ! -e ubuntu-12.04.4-desktop-amd64.iso ]; then
        wget http://releases.ubuntu.com/12.04/ubuntu-12.04.4-desktop-amd64.iso
    fi
    sudo uck-remaster-unpack-iso ubuntu-12.04.4-desktop-amd64.iso
    sudo uck-remaster-unpack-rootfs
fi

# setup custom disk
cat <<EOF | sudo uck-remaster-chroot-rootfs
set -x
set -e

if [ ! ${DEBUG} ]; then
whoami
if [ \`grep universe /etc/apt/sources.list | wc -l\` -eq 0 ]; then
  echo "
#
deb http://archive.ubuntu.com/ubuntu/ precise main universe
deb http://security.ubuntu.com/ubuntu/ precise-security main universe
deb http://archive.ubuntu.com/ubuntu/ precise-updates main universe
#
deb http://archive.ubuntu.com/ubuntu/ precise main multiverse
deb http://security.ubuntu.com/ubuntu/ precise-security main multiverse
deb http://archive.ubuntu.com/ubuntu/ precise-updates main multiverse
" >> /etc/apt/sources.list;
fi
cat /etc/apt/sources.list

# omajinai
apt-get update
apt-get -y upgrade

# install ros
wget --no-check-certificat -O /tmp/jsk.rosbuild https://raw.github.com/jsk-ros-pkg/jsk_common/master/jsk.rosbuild
chmod u+x /tmp/jsk.rosbuild
/tmp/jsk.rosbuild hydro setup-ros
apt-get -y install ros-hydro-rtmros-nextage
apt-get -y install ros-hydro-rtmros-hironx
apt-get -y install ros-hydro-denso
apt-get -y install ros-hydro-common-tutorials
apt-get -y install ros-hydro-turtlebot-viz
apt-get -y install ros-hydro-turtlebot-simulator
apt-get -y install ros-hydro-turtlebot-apps
apt-get -y install ros-hydro-turtlebot

# install emacs
apt-get -y install emacs

# install chromium
apt-get -y install chromium-browser

# install gnome-open
apt-get -y install libgnome2.0

# for japanese environment
apt-get -y install language-pack-gnome-ja latex-cjk-japanese xfonts-intl-japanese

fi # ( [ ! ${DEBUG} ] )

# setup catkin
mkdir -p /home/ubuntu/catkin_ws/src
cd /home/ubuntu/catkin_ws/src
wstool init || echo "already initilized"
wstool set roscpp_tutorials https://github.com/ros/ros_tutorials.git --git -y || echo "already configured"
wstool update
cd -
chown -R 999.999 /home/ubuntu

# desktop settings
if [ ! -e /home/ubuntu/tork-ros.png ]; then
  wget https://github.com/tork-a/live-cd/raw/master/tork-ros.png -O /home/ubuntu/tork-ros.png
fi
## dbus-launch --exit-with-session gsettings set org.gnome.desktop.background picture-uri file:///home/ubuntu/tork-ros.png
echo "
[org.gnome.desktop.background]
picture-uri='file:///home/ubuntu/tork-ros.png'
" > /usr/share/glib-2.0/schemas/10_local-desktop-background.gschema.override

# setup keyboard
# dbus-launch --exit-with-session gsettings set org.gnome.libgnomekbd.keyboard options "['ctrl\tctrl:swapcaps']"
echo "
[org.gnome.libgnomekbd.keyboard]
options=['ctrl\tctrl:nocaps']
" > /usr/share/glib-2.0/schemas/10_local-libgnomekbd-keyboard.gschema.override

# add gnome-terminal icon
dbus-launch --exit-with-session gsettings set com.canonical.Unity.Launcher favorites "\$(gsettings get com.canonical.Unity.Launcher favorites | sed "s/, *'gnome-terminal.desktop' *//g" | sed "s/'gnome-terminal.desktop' *, *//g" | sed -e "s/]$/, 'gnome-terminal.desktop']/")"
gsettings get com.canonical.Unity.Launcher favorites

# add chromium-browser.desktop
dbus-launch --exit-with-session gsettings set com.canonical.Unity.Launcher favorites "\$(gsettings get com.canonical.Unity.Launcher favorites | sed "s/, *'chromium-browser.desktop' *//g" | sed "s/'chromium-browser.desktop' *, *//g" | sed -e "s/]$/, 'chromium-browser.desktop']/")"

echo "
[com.canonical.Unity.Launcher]
favorites=\`gsettings get com.canonical.Unity.Launcher favorites\`
" > /usr/share/glib-2.0/schemas/10_local-unity-launcher.gschema.override

## recompile schemas file
glib-compile-schemas /usr/share/glib-2.0/schemas/

## http://wiki.backbox.org/index.php/Customize_the_Live_DVD
# Some of the services might restart while updating. We recommend to stop them in order to make a fast booting ISO.

service apache2 stop
service tor stop
service polipo stop
service greenbone-security-assistant stop
service openvas-administrator stop
service openvas-manager stop
service openvas-scanner stop
update-rc.d -f apache2 remove
update-rc.d -f tor remove
update-rc.d -f polipo remove
update-rc.d -f greenbone-security-assistant remove
update-rc.d -f openvas-administrator remove
update-rc.d -f openvas-manager remove
update-rc.d -f openvas-scanner remove

# Now you can update or remove packages, customize everything you want.. Before closing the ISO, it would be best to remove any temporary files which are no longer needed

apt-get clean
rm -f /etc/apt/*.save
rm -f /etc/apt/sources.list.d/*.save
rm -f /var/crash/*
rm -rf /tmp/* ~/.bash_history
rm /etc/hosts
rm /etc/resolv.conf
rm /var/lib/dbus/machine-id

# Now you can make a new initramfs image and exit:

update-initramfs -u

EOF

if [ ! ${DEBUG} ]; then
    sudo rm -f ~/tmp/remaster-iso/casper/initrd.lz
    sudo cp ~/tmp/remaster-root/boot/initrd.img-*  ~/tmp/remaster-iso/casper/initrd.gz
    sudo cp ~/tmp/remaster-root/boot/vmlinuz-*  ~/tmp/remaster-iso/casper/vmlinuz

    # pack file system
    sudo uck-remaster-pack-rootfs -c

    # create iso
    FILENAME=tork-ubuntu-ros-12.04-amd64-`date +%Y%m%d`.iso
    sudo uck-remaster-pack-iso $FILENAME -g -d "TORK Ubuntu/ROS Linux"
    \cp -f ~/tmp/remaster-new-files/$FILENAME .
fi





