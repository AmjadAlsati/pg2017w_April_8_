#!/bin/bash
path=$1
echo "Time,CPU1,CPU2,CPU3,CPU4" > $path
while true
do
	now=$(date +%s%3N) # Linux epoch time in ms
	loads=($(mpstat -P ALL 1 1 | awk '/Average:/ && $2 ~ /[0-9]/ {print $3}'))
	echo "${now},${loads[0]},${loads[1]},${loads[2]},${loads[3]}" >> $path

	#(echo “%CPU %MEM ARGS $(date)” && ps -e -o pcpu,pmem,args –sort=pcpu | cut -d“ ” -f1-5 | tail) >> ps.log
done
