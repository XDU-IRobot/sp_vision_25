#!/bin/bash
set -x
echo "PATH=$PATH"
#export OPENCV_VIDEOIO_PRIORITY_MSMF=0
export DISPLAY=:0
export XAUTHORITY=/home/nvidia/.Xauthority
#export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
#sleep 5
#echo "autostart $(date)">> /tmp/vision_autostart.log
conda deactivate
cd /home/nvidia/sp_vision_25 || exit
./build/uav_debug_jetsn configs/complete_template.yaml
#mkdir -p logs
#/usr/bin/screen \
#    -S vision \
#    -L \
#    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
#    -d \
#    -m \
#    bash -c "./uav_debug_jetsn ../configs/.yaml "
