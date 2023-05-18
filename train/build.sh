#!/bin/bash

docker build -f ./docker/Dockerfile \
       --rm \
       -t train-object-detection-from-images \
       .
