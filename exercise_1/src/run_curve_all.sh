#!/bin/bash

# run python script many times
for file in $@; do python3 $1 -ng -s 0.1 -x vd_double_track/x_vec/x_m -y vd_double_track/x_vec/y_m vd_double_track/x_vec/x_m -- $file; done