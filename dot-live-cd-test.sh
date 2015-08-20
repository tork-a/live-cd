#!/bin/bash

function check-network {
    echo "=== Check network reachability"
    dig ros.org
}

function check-rospack {
    echo "*** Check roscpp installed"
    source .bashrc
    rospack find roscpp
}

sudo mount /dev/sda /mnt
sudo touch /mnt/result.txt
sudo chmod a+rw /mnt/result.txt
{
(check-network && echo "=== OK ===") || echo "=== NG ==="
(check-rospack && echo "=== OK ===") || echo "=== NG ==="
echo "===DONE==="
} 2>&1 | tee -a /mnt/result.txt


echo " ==== cat .bashrc ==== " | tee -a /mnt/result.txt
cat .bashrc | tee -a /mnt/result.txt

echo " ==== env ==== " | tee -a /mnt/result.txt
env | tee -a /mnt/result.txt

# done

