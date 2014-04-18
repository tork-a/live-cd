#!/bin/bash
set -x
set -e

function usage {
    set +x
    echo >&2 "usage: $0"
    echo >&2 "          [--help] print this message"
    echo >&2 "          [--debug] debug mode "
    echo >&2 "          [--rosdistro (hydro|indigo)] "
    exit 0
}

OPT=`getopt -o hdr: -l help,debug,rosdistro: -- $*`
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
        -r| --rosdistro) ROSDISTRO=$2; shift 2;;
        --) shift; break;;
    esac
done

case $ROSDISTRO in
    hydro ) ISO=ubuntu-12.04.4-desktop-amd64.iso;;
    indigo) ISO=ubuntu-14.04-desktop-amd64.iso;;
    *) echo "[ERROR] Unsupported ROSDISTRO $ROSDISTRO"; exit;;
esac
REV=`echo ${ISO} | sed "s/ubuntu-\([0-9]*.[0-9]*\).*/\\1/"`

# init stuff
if [ ! ${DEBUG} ]; then
    sudo uck-remaster-clean
    if [ ! -e ${ISO} ]; then
        wget http://releases.ubuntu.com/${REV}/${ISO}
    fi
    sudo uck-remaster-unpack-iso ${ISO}
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
/tmp/jsk.rosbuild $ROSDISTRO setup-ros

if [ ${ROSDISTRO} == "hydro" ]; then
apt-get -y install ros-$ROSDISTRO-rtmros-nextage
apt-get -y install ros-$ROSDISTRO-rtmros-hironx
apt-get -y install ros-$ROSDISTRO-denso
apt-get -y install ros-$ROSDISTRO-common-tutorials
apt-get -y install ros-$ROSDISTRO-turtlebot-viz
apt-get -y install ros-$ROSDISTRO-turtlebot-simulator
apt-get -y install ros-$ROSDISTRO-turtlebot-apps
apt-get -y install ros-$ROSDISTRO-turtlebot
fi

# install emacs
apt-get -y install emacs

# install chromium
apt-get -y install chromium-browser

# install gnome-open
apt-get -y install libgnome2.0

# for japanese environment
if [ ${ROSDISTRO} == "hydro" ]; then
apt-get -y install language-pack-gnome-ja latex-cjk-japanese xfonts-intl-japanese
fi

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


EOF

if [ ! ${DEBUG} ]; then
     # pack file system
    sudo uck-remaster-pack-rootfs

    # create iso
    DATE=`date +%Y%m%d`
    FILENAME=tork-ubuntu-ros-${REV}-amd64-${DATE}.iso
    sudo uck-remaster-pack-iso $FILENAME -g --description="TORK Ubuntu/ROS Linux (${DATE})"
    \cp -f ~/tmp/remaster-new-files/$FILENAME .
fi





