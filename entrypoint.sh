#!/bin/bash
dronekit-sitl --list
dronekit-sitl copter-3.3 --home=50.450739,30.461242,0,280 &

#mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 &

# python first.py --connect udp:127.0.0.1:14551
python second.py

