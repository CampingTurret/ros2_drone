#!/bin/bash

# sudo docker run -it --rm --network=host elijahanghw/ros_imav:latest

sudo docker run -it --rm \
  --user $(id -u):$(id -g) \
  --network=host \
  --privileged \
  --device /dev/gpiomem4 \
  --device /dev/mem \
  -v $(pwd)/logs:/root/logs \
  elijahanghw/ros_imav:latest
