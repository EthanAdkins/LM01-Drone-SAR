#!/bin/bash
export PX4_SIM_HOST_ADDR=$1
#socat UDP4-RECVFROM:5600,fork UDP4-SENDTO:${HOST_IP}:5600 &#!/bin/bash
initial=5760
instance=$2
mavlink_port=$(($initial+$instance))

python3 /usr/local/bin/mavproxy.py --daemon --master=udpin:127.0.0.1:14550 --out=tcpin:0.0.0.0:$mavlink_port 2>&1 > /dev/null &
#cd /src/PX4-Autopilot && HEADLESS=1 make px4_sitl_default none_iris


build_path=/src/PX4-Autopilot/build/px4_sitl_default
export PX4_SIM_MODEL=none_iris

working_dir="$build_path/instance_$n"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

pushd "$working_dir" &>/dev/null
echo "starting instance $n in $(pwd)"
$build_path/bin/px4 -i $instance -d "$build_path/etc"
popd &>/dev/null


#cd /src/PX4-Autopilot/Tools/simulation/
#./sitl_multiple_run.sh 2
# ./test.sh 172.23.0.1 1
