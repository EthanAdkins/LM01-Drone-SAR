#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PX4_PATH = ~/PX4-Autopilot

# Function to ask user for the number of containers with a maximum limit of 10
ask_for_container_count() {
    while true; do
        echo "Enter the number of containers (max 10):"
        read container_count

        # Check if input is an integer and less than or equal to 10
        if [[ "$container_count" =~ ^[0-9]+$ ]] && [ "$container_count" -le 10 ]; then
            break
        else
            echo "Please enter a valid number (1-10)."
        fi
    done
}

ask_for_container_count

# Start the Docker Compose file
cat <<EOF > docker-compose.yml
version: '3.8'

services:
EOF

# Function to generate container configuration
generate_container_config() {
    local container_number=$1
    local udp_port=$((14570 + container_number - 1))
    local tcp_port=$((5760 + container_number - 1))
    local instance=$((container_number - 1))

    cat <<EOF
  px4-dev-ros-$container_number:
    image: njbrown09/lm01-drone-sar:latest
    privileged: true
    stdin_open: true
    tty: true
    environment:
      - PX4_HOME_LAT=42.3898
      - PX4_HOME_LON=-71.1476
      - PX4_HOME_ALT=14.2
      - PX4_INSTANCE=$instance
      - PX4_SIM_HOST_ADDR=172.23.0.1
      - DISPLAY=:0
    volumes:
      - /home/nick/PX4-Autopilot:/src/PX4-Autopilot:rw
      - /tmp/.X11-unix:/tmp/.X11-unix:ro
    ports:
      - "$udp_port:$udp_port/udp"
      - "$tcp_port:$tcp_port/tcp"
    networks:
      - px4_network

EOF
}

# Generate configurations for each container
for (( i=1; i<=container_count; i++ ))
do
    generate_container_config $i >> docker-compose.yml
done

# Add network configuration
cat <<EOF >> docker-compose.yml
networks:
  px4_network:
    driver: bridge
EOF

echo "Docker Compose file generated: docker-compose.yml"

# Create containers
echo -e "\e[1;32mCreating containers using Docker Compose...\e[0m"
docker-compose -f docker-compose.yml up
