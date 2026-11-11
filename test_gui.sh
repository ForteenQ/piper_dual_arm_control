#!/usr/bin/env bash
for var in $(systemctl --user show-environment); do
    export "$var"
done
xhost +
glxinfo -B 2>&1 | head -n 15
