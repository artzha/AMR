#!/bin/bash
USER=$(whoami)

docker run -it --net=host \
    --gpus all \
    -v $(pwd)/amrl_maps:/root/amrl_maps \
    -v $(pwd)/amrl_msgs:/root/amrl_msgs \
    -v $(pwd)/cs393r_starter:/root/cs393r_starter \
    -v $(pwd)/ut_automata:/root/ut_automata cs393r_starter
