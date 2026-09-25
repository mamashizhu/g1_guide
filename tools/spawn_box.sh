#!/usr/bin/env bash
# 在展馆里放一个红色箱子障碍（高 1.8m，导航雷达能扫到）
# 用法：bash spawn_box.sh <名字> <x> <y>
name=${1:-blocker}; x=${2:-0}; y=${3:--2.5}
sdf="<?xml version='1.0'?><sdf version='1.9'><model name='$name'><static>true</static><link name='l'><collision name='c'><geometry><box><size>0.8 0.8 1.8</size></box></geometry></collision><visual name='v'><geometry><box><size>0.8 0.8 1.8</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"
ros2 run ros_gz_sim create -world exhibition -name "$name" -x "$x" -y "$y" -z 0.95 -string "$sdf"
