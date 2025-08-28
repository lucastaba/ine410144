#!/bin/bash

# run python script many times
for file in $@; do python3 $1 -ng -s 0.1 -x vd_double_track/imr/tire_longitudinal_slip/front_left -y vd_double_track/imr/tire_forces_N/front_left/longitudinal vd_double_track/imr/tire_forces_N/front_left/lateral -- $file; done