#!/bin/bash
set -x
set -e

function usage {
    set +x
    echo >&2 "usage: $0"
    echo >&2 "          [--help] print this message"
    echo >&2 "          [--debug] debug mode "
    echo >&2 "          [--local-repo] keep deb as local repo "
    echo >&2 "          [--rosdistro (hydro|indigo)] "
    exit 0
}

OPT=`getopt -o hdlr: -l help,debug,local-repo,rosdistro: -- $*`
if [ $? != 0 ]; then
    usage
fi

ROSDISTRO=hydro
eval set -- $OPT
while [ -n "$1" ] ; do
    echo $1
    case $1 in
        -h| --help) usage; shift;;
        -d| --debug) DEBUG=TRUE; shift;;
        -l| --local-repo) ENABLE_LOCAL_REPOSITORY=TRUE; shift;;
        -r| --rosdistro) ROSDISTRO=$2; shift 2;;
        --) shift; break;;
    esac
done

case $ROSDISTRO in
    hydro ) ISO=ubuntu-12.04.5-desktop-amd64.iso;;
    indigo) ISO=ubuntu-14.04.3-desktop-amd64.iso;;
    *) echo "[ERROR] Unsupported ROSDISTRO $ROSDISTRO"; exit;;
esac
REV=`echo ${ISO} | sed "s/ubuntu-\([0-9]*.[0-9]*\).*/\\1/"`

DATE=`date +%Y%m%d_%H%M%S`

# init stuff
if [ ! ${DEBUG} ]; then
    sudo uck-remaster-clean
    if [ ! -e /tmp/${ISO} ]; then
        wget -q http://releases.ubuntu.com/${REV}/${ISO} -O /tmp/${ISO}
    fi
    sudo uck-remaster-unpack-iso /tmp/${ISO}
    sudo uck-remaster-unpack-rootfs
fi

# setup custom disk
cat <<EOF | sudo uck-remaster-chroot-rootfs
set -x
set -e

umask 022

if [ ! ${DEBUG} ]; then
whoami
if [ \`grep universe /etc/apt/sources.list | wc -l\` -eq 0 ]; then
  echo "
#
deb http://archive.ubuntu.com/ubuntu/ \`lsb_release -cs\` main universe
deb http://security.ubuntu.com/ubuntu/ \`lsb_release -cs\`-security main universe
deb http://archive.ubuntu.com/ubuntu/ \`lsb_release -cs\`-updates main universe
#
deb http://archive.ubuntu.com/ubuntu/ \`lsb_release -cs\` main multiverse
deb http://security.ubuntu.com/ubuntu/ \`lsb_release -cs\`-security main multiverse
deb http://archive.ubuntu.com/ubuntu/ \`lsb_release -cs\`-updates main multiverse
" >> /etc/apt/sources.list;
fi
cat /etc/apt/sources.list
([ -e /etc/apt/sources.list~ ] && rm -f /etc/apt/sources.list~; ls /etc/apt/)

# install ros
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu \`lsb_release -cs\` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update
echo "hddtemp hddtemp/daemon boolean false" | sudo debconf-set-selections
apt-get -y -q install ros-$ROSDISTRO-desktop-full ros-$ROSDISTRO-catkin  ros-$ROSDISTRO-rosbash
apt-get -y -q install python-wstool python-rosdep python-catkin-tools
apt-get -y -q install aptitude git ntp emacs vim

# mongodb hack for 14.04
if [ ${ROSDISTRO} == "indigo" ]; then
  echo "mongodb:x:130:65534::/home/mongodb:/bin/false" >> /etc/passwd
  echo "mongodb:x:130:mongodb" >> /etc/group
  apt-get install -q -y mongodb mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"
  sed -i -r 's/invoke-rc.d/#&/' /var/lib/dpkg/info/mongodb-server.postinst
  apt-get install -q -y mongodb mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"
fi

# rosdep
rosdep init; rosdep update || echo "ok"

# make home directory
mkdir -p /home/ubuntu/
chown -R 999.999 /home/ubuntu/
cp /etc/skel/.??* /home/ubuntu
chown -R 999.999 /home/ubuntu/.??*
echo "
# ROS setup
source /opt/ros/$ROSDISTRO/setup.bash

# This file is created on ${DATE}
" >> /home/ubuntu/.bashrc
HOME=/home/ubuntu rosdep update
chown -R 999.999 /home/ubuntu/.ros

# make catkin workspace
mkdir -p /home/ubuntu/catkin_ws/src
cd /home/ubuntu/catkin_ws
wstool init src || echo "already initilized"

# setup ros_tutorials
wstool set -t src roscpp_tutorials https://github.com/ros/ros_tutorials.git --git -y || echo "already configured"
# for baxter seminar
wstool merge -t src https://raw.github.com/tork-a/baxter_seminar/master/baxter_seminar.rosinstall
# For TORK ROS workshop
wstool set -t src tork-a/ros_seminar https://github.com/tork-a/ros_seminar.git --git -y || echo "tork-a/ros_seminar already configured"

# update and install
wstool update -t src
rosdep install -r -n -y --rosdistro $ROSDISTRO --from-paths src --ignore-src
# compile with catkin
. /opt/ros/$ROSDISTRO/setup.sh
catkin build -p 1 --no-status
cd -
chown -R 999.999 /home/ubuntu/catkin_ws

# For turtlebot
apt-get -y -q install ros-$ROSDISTRO-turtlebot-simulator
apt-get -y -q install ros-$ROSDISTRO-turtlebot-apps
apt-get -y -q install ros-$ROSDISTRO-turtlebot
apt-get -y -q install ros-$ROSDISTRO-turtlebot-viz
# For kobuki
apt-get -y -q install ros-$ROSDISTRO-kobuki-desktop
# For devices
apt-get -y -q install ros-$ROSDISTRO-dynamixel-motor
apt-get -y -q install ros-$ROSDISTRO-libuvc-camera
apt-get -y -q install ros-$ROSDISTRO-uvc-camera
apt-get -y -q install ros-$ROSDISTRO-ar-track-alvar
apt-get -y -q install ros-$ROSDISTRO-openni2-launch
apt-get -y -q install ros-$ROSDISTRO-audio-common
# For moveit
apt-get -y -q install ros-$ROSDISTRO-moveit-ikfast
apt-get -y -q install ros-$ROSDISTRO-moveit-full-pr2
apt-get -y -q install ros-$ROSDISTRO-industrial-desktop
# qt-build
apt-get -y -q install ros-$ROSDISTRO-qt-build
# For Denso
apt-get -y -q install ros-$ROSDISTRO-denso
# For Nextage
apt-get -y -q install ros-$ROSDISTRO-rtmros-hironx ros-$ROSDISTRO-rtmros-nextage

if [ ${ROSDISTRO} == "hydro" ]; then
# RTM, Hiro-NXO
#apt-get -y -q install ros-$ROSDISTRO-hironx-tutorial

#rosemacs
apt-get -y -q install rosemacs-el
fi # hydro
if [ ${ROSDISTRO} == "indigo" ]; then
apt-get -y -q install ros-$ROSDISTRO-rosemacs

# hakuto
apt-get -y -q install ros-$ROSDISTRO-hakuto
fi


# install chromium
apt-get -y -q install chromium-browser

# install gnome-open
apt-get -y -q install libgnome2.0

# install freecad
apt-get -y -q install freecad

# for japanese environment
apt-get -y -q install language-pack-gnome-ja latex-cjk-japanese xfonts-intl-japanese

# development/debug tools
apt-get -y -q install sshpass synaptic

# fix resolve conf (https://github.com/tork-a/live-cd/issues/8)
rm -fr /etc/resolv.conf
apt-get -y -q install debconf-utils
echo "resolvconf resolvconf/linkify-resolvconf boolean true" | debconf-set-selections -
dpkg-reconfigure -fnoninteractive resolvconf

fi # ( [ ! ${DEBUG} ] )

# desktop settings
if [ ! -e /home/ubuntu/tork-ros.png ]; then
  wget https://github.com/tork-a/live-cd/raw/master/tork-ros.png -O /home/ubuntu/tork-ros.png
  chown -R 999.999 /home/ubuntu/tork-ros.png
fi

## dbus-launch --exit-with-session gsettings set org.gnome.desktop.background picture-uri file:///home/ubuntu/tork-ros.png
echo "
[org.gnome.desktop.background]
picture-uri='file:///home/ubuntu/tork-ros.png'
" > /usr/share/glib-2.0/schemas/99_local-desktop-background.gschema.override

# setup keyboard
# dbus-launch --exit-with-session gsettings set org.gnome.libgnomekbd.keyboard options "['ctrl\tctrl:swapcaps']"
echo "
[org.gnome.libgnomekbd.keyboard]
options=['ctrl\tctrl:nocaps']
" > /usr/share/glib-2.0/schemas/99_local-libgnomekbd-keyboard.gschema.override

# add gnome-terminal icon
dbus-launch --exit-with-session gsettings set com.canonical.Unity.Launcher favorites "\$(gsettings get com.canonical.Unity.Launcher favorites | sed "s/, *'gnome-terminal.desktop' *//g" | sed "s/'gnome-terminal.desktop' *, *//g" | sed -e "s/]$/, 'gnome-terminal.desktop']/")"
gsettings get com.canonical.Unity.Launcher favorites

# add chromium-browser.desktop
dbus-launch --exit-with-session gsettings set com.canonical.Unity.Launcher favorites "\$(gsettings get com.canonical.Unity.Launcher favorites | sed "s/, *'chromium-browser.desktop' *//g" | sed "s/'chromium-browser.desktop' *, *//g" | sed -e "s/]$/, 'chromium-browser.desktop']/")"

echo "
[com.canonical.Unity.Launcher]
favorites=\`gsettings get com.canonical.Unity.Launcher favorites\`
" > /usr/share/glib-2.0/schemas/99_local-unity-launcher.gschema.override

## recompile schemas file
glib-compile-schemas /usr/share/glib-2.0/schemas/

## write test code
if [ ! -e /home/ubuntu/.live-cd-test.sh ]; then
  echo "`cat dot-live-cd-test.sh`" >> /home/ubuntu/.live-cd-test.sh
  chown -R 999.999 /home/ubuntu/.live-cd-test.sh
  chmod a+x /home/ubuntu/.live-cd-test.sh
fi

EOF

if [ ! ${DEBUG} ]; then
     # pack file system
    sudo uck-remaster-pack-rootfs

    # create local repository
    if [ ${ENABLE_LOCAL_REPOSITORY} ]; then
      sudo mkdir -p ~/tmp/remaster-iso/repository
      sudo cp -r ~/tmp/remaster-apt-cache/archives ~/tmp/remaster-iso/repository/binary
      sudo chmod a+rx ~/tmp/remaster-iso/repository/binary/
      sudo su -c "cd ${HOME}/tmp/remaster-iso/repository/; dpkg-scanpackages binary /dev/null | gzip -9c > binary/Packages.gz"
   fi

    ## update boot option
    sudo su -c "cd ${HOME}/tmp/remaster-iso/isolinux; sed -i 's/quiet splash//' txt.cfg"
    sudo su -c "cd ${HOME}/tmp/remaster-iso/isolinux; sed -i 's/^/#/' isolinux.cfg"
    sudo su -c "cd ${HOME}/tmp/remaster-iso/isolinux; echo 'include txt.cfg' >> isolinux.cfg"


    # create iso
    if [ ${ENABLE_LOCAL_REPOSITORY} ]; then
        FILENAME=tork-ubuntu-ros-${REV}-local-repo-amd64-${DATE}.iso
    else
        FILENAME=tork-ubuntu-ros-${REV}-amd64-${DATE}.iso
    fi
    DATE=`date +%m%d`
                                              #1234 56789012345 678901 2 3 456789012
    sudo uck-remaster-pack-iso $FILENAME -g -d=TORK\ Ubuntu/ROS\ Linux\ \(${DATE}\)
    sudo cp -f ~/tmp/remaster-new-files/$FILENAME .
    sudo chown jenkins.jenkins $FILENAME
fi





