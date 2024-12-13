#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./RobustFNN4Quasiperiodic_rt
else
	EXEC=./RobustFNN4Quasiperiodic_nrt
fi

$EXEC -n x8_0 -a 127.0.0.1 -p 9000 -l ./ -x setup_x8.xml -t x8_simu
