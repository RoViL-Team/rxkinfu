#!/bin/sh

./rxkinfu -imucam_reader "$@" -bf -bc -br 200,0,0,0,0,0  -bo 0,-1,0 -pause -drawcam -camaa -1,1,0,0 -ndv -nrv -downgrav -drawdown -mvfd -mvcnn -mvcv -bubcam -bubcamloc 10,-4,-4 -campos 2,1.5,1.5 -vs 4
