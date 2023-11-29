#!/bin/bash
export PX4_SIM_HOST_ADDR=$1
initial=5760
instance=$2
mavlink_port=$(($initial+$instance))

#Start MavProxy
python3 /usr/local/bin/mavproxy.py --daemon --master=udpin:127.0.0.1:14550 --out=tcpin:0.0.0.0:$mavlink_port 2>&1 > /dev/null &

#Start PX4
build_path=/src/PX4-Autopilot/build/px4_sitl_default
export PX4_SIM_MODEL=none_iris

working_dir="$build_path/instance_$n"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
pushd "$working_dir" &>/dev/null

echo "starting instance $n in $(pwd)"
$build_path/bin/px4 -i $instance -d "$build_path/etc"
popd &>/dev/null


