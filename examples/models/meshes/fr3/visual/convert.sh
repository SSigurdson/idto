#!/bin/bash

for f in ./*.dae
do
    name=$(basename $f .dae)
    echo $name
    meshlabserver -i $name.dae -o $name.obj
done
