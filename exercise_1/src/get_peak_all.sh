#!/bin/bash

# run python script many times
for file in $@; do python3 $1 -ng -ns -pk -s 0.1 -y vd_double_track/x_vec/v_x_mps vd_double_track/x_vec/v_x_mps -- $file; done