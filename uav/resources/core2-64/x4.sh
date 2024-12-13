#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./RobustFNN4Quasiperiodic_rt
else
	EXEC=./RobustFNN4Quasiperiodic_nrt
fi

$EXEC -n x4_0 -a 127.0.0.1 -p 9000 -l /tmp -x setup_x4.xml -t x4_simu 
