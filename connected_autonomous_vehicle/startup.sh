#!/bin/sh
source /usr/local/bin/virtualenvwrapper.sh
workon iot
cd /home/jetson/Projects/slamware/
python3 /home/jetson/Projects/slamware/cav1.py
