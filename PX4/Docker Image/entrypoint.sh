#!/bin/bash
export PX4_SIM_HOST_ADDR=$1
initial=5760
instance=$2
mavlink_port=$(($initial+$instance))

#Start MavProxy
echo 'Starting mavproxy'
python3 /usr/local/bin/mavproxy.py --daemon --master=udpin:127.0.0.1:14550 --out=tcpin:0.0.0.0:$mavlink_port 2>&1 > /dev/null &

#Start PX4
build_path=/src/PX4-Autopilot/build/px4_sitl_default
export PX4_SIM_MODEL=none_iris

working_dir="$build_path/instance_$instance"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
pushd "$working_dir" &>/dev/null

echo "starting instance $instance in $(pwd)"
$build_path/bin/px4 -i $instance -d "$build_path/etc"
popd &>/dev/null

echo "Compiling manually"
cd /src/PX4-Autopilot

#make px4_sitl_default none_iris

#python3 /usr/local/bin/mavproxy.py --daemon --master=udpin:127.0.0.1:14550 --out=tcpin:0.0.0.0:4560 2>&1 > /dev/null &
# ./PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0 -d "/home/nick/PX4-Autopilot/build/px4_sitl_default/etc"
# 
# python3 /usr/local/bin/mavproxy.py --daemon --master=udpin:127.0.0.1:14550 --out=tcpin:0.0.0.0:4561 2>&1 > /dev/null &
# ./PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1 -d "/home/nick/PX4-Autopilot/build/px4_sitl_default/etc"
