#!/bin/sh
case "$1" in 
start)
cd /home/nvidia/
./jetson_clocks.sh > /var/log/startup2019.txt
mkdir /tmp/caminfo
python assign_camera.py >> /var/log/startup2019.txt
;;
stop|restart|reload|force-reload)
;;
esac

exit 0
