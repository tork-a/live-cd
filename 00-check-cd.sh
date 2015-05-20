#!/bin/bash
set -x
set -e

# setup iso
iso=${1:-tork-ubuntu-ros-12.04-amd64-*.iso}

# create usb memory
rm -fr /tmp/disk.usb
dd if=/dev/zero of=/tmp/disk.usb bs=1024k count=32
echo "n
p
1


q
"|/sbin/fdisk /tmp/disk.usb; /sbin/mkfs.ext2 -F /tmp/disk.usb

# run qemu
qemu-system-x86_64 -monitor telnet:0.0.0.0:1025,server,nowait -usb -cdrom $iso -boot d -m 4096 -enable-kvm -vga std & # -vnc :70  &

qemupid=$!

sleep 120 # whope this is ok, since -serial file does not work....

exec 3<>/dev/tcp/localhost/1025
sleep 5

echo "sendkey tab">&3
sleep 5

echo "sendkey ret">&3
sleep 20

echo "usb_add disk:/tmp/disk.usb">&3
sleep 10

echo "sendkey ctrl-alt-t">&3
sleep 10

echo "sendkey l">&3;   sleep 2
echo "sendkey s">&3; sleep 2
echo "sendkey spc">&3;   sleep 2
echo "sendkey minus">&3;     sleep 2
echo "sendkey a">&3;     sleep 2
echo "sendkey l">&3;     sleep 2
echo "sendkey ret">&3;   sleep 2
sleep 10

echo "sendkey dot">&3;   sleep 2
echo "sendkey slash">&3; sleep 2
echo "sendkey dot">&3;   sleep 2
echo "sendkey l">&3;     sleep 2
echo "sendkey i">&3;     sleep 2
echo "sendkey v">&3;     sleep 2
echo "sendkey tab">&3;   sleep 2
echo "sendkey ret">&3;   sleep 2
sleep 30

echo "screendump desktop.ppm">&3;   sleep 2
convert desktop.ppm desktop.png
chmod a+rw desktop.ppm desktop.png

echo "sendkey s">&3;   sleep 1
echo "sendkey u">&3;   sleep 1
echo "sendkey d">&3;   sleep 1
echo "sendkey o">&3;   sleep 1
echo "sendkey spc">&3;   sleep 1
echo "sendkey h">&3;   sleep 1
echo "sendkey a">&3;   sleep 1
echo "sendkey l">&3;   sleep 1
echo "sendkey t">&3;   sleep 1
echo "sendkey ret">&3;   sleep 1
sleep 10

kill -KILL $qemupid

mkdir /tmp/mount.$$
mount /tmp/disk.usb /tmp/mount.$$
cat /tmp/mount.$$/result.txt
ls -al /tmp/mount.$$/

# if DONE is not found, exit 1
grep -c "===DONE===" /tmp/mount.$$/result.txt || (umount /tmp/mount.$$; exit 1)
# if nG is found, exit 1
grep -c "=== NG ===" /tmp/mount.$$/result.txt && (umount /tmp/mount.$$; exit 1)

# exit ok
umount /tmp/mount.$$
exit 0

