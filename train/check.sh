#!/bin/bash

echo $(docker ps -q --filter "name=thk-train-pytorch-object-detection" | wc -l)
