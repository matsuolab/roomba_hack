#!/bin/sh

Xvfb :0 -screen 0 1400x900x24 &

x11vnc -display :0 -passwd pass -forever &

sleep 3s && icewm-session &

wait