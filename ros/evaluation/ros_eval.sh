#!/bin/bash

path=$1
echo ">> Create result directory ${path}"
mkdir -p $path

ros=false

function wait {
     echo ">> Idle for ${1}s"
     sleep $1
}

function start_cpu_dump {
    dumpfile=$1/cpuload.csv
    echo ">> Start dumping CPU load to ${dumpfile}"
    ~/dump_cpu_load.sh $dumpfile &
}

function init_run {
    idle_period=$1
    name=$2
    resultdir=$path/$name
    mkdir -p $resultdir
    start_cpu_dump $resultdir
    wait $idle_period
    if $ros 
    then
        echo ">> Start ros core"
        roscore&
        sleep 5
    fi
}

function start_talker {
    talker_name=$1
    topic=$2
    rate=$3
    size=$4
    echo ">> Start talker ${talker_name} at rate ${rate} with msg size ${size} on topic ${topic}"
    if $ros 
    then
        rosrun delay_tests talker.py -t $topic -n $talker_name -r $rate $size &
    else
        python talker_zmq.py -p $topic -n $talker_name -r $rate $size &
    fi
}

function start_listener {
    listener_name=$1
    topic=$2
    output_path=$3

    echo ">> Start listener ${listener_name} on topic ${topic}"
    echo ">> Result file stored in ${output_path}"

    if $ros
    then
        rosrun delay_tests listener.py -n $listener_name -t $topic -o $output_path &
    else
        echo "Port: ${port}"
        python listener_zmq.py -n $listener_name -p $topic -o $output_path &
    fi
}

function stop_processes {
    echo ">> Stop processes"
    if $ros
    then
        rosnode kill -a
        sleep 5
        killall roscore
    else
        killall python #diiiirtyyyy
    fi
    sleep 5
}

function stop_cpu_dump {
    echo ">> Stop CPU load dumping"
    killall dump_cpu_load.sh
    sleep 5
}

function realistic_test {
    name="realistic_conditions"
    echo ">>>> Start Run  \"${name}\" <<<<"
    init_run 30 $name

    if $ros
    then
        topic1="topic1"
        topic2="topic2"
        topic3="topic3"
    else
        topic1="8080"
        topic2="8081"
        topic3="8082"
    fi

    rate1=50
    size1=1000000
    rate2=50
    size2=1000
    rate3=50
    size3=1000
    start_talker "talker1" $topic1 $rate1 $size1
    start_talker "talker2" $topic2 $rate2 $size2
    start_talker "talker3" $topic3 $rate3 $size3
    start_listener "listener1" $topic1 $resultdir/listener1_rate$rate1\_size$size1.csv
    start_listener "listener2" $topic2 $resultdir/listener2_rate$rate2\_size$size2.csv
    start_listener "listener3" $topic2 $resultdir/listener3_rate$rate2\_size$size2.csv
    start_listener "listener4" $topic3 $resultdir/listener4_rate$rate3\_size$size3.csv
    wait 60
    stop_processes
    wait 30
    stop_cpu_dump
    echo ">>>> Done with run ${name} <<<<"
}

function n_talkers_m_listeners {
    #Start n talkers, EACH having m listeners

    num_talkers=$1
    num_listeners=$2
    rate=$3
    size=$4

    name="${num_talkers}_talkers_${num_listeners}_listeners"
    echo ">>>> Start Run ${name} <<<<"
    init_run 30 $name
    if [ "$ros" = false ]; then
        port=8080
    fi
    for ((i=1; i<=$num_talkers; i++))
    do
      if $ros
      then
          topic="topic${i}"
      else
          topic=$(($port+i))
      fi

      start_talker "talker${i}" $topic $rate $size
      for ((j=1; j<=$num_listeners; j++))
      do
        start_listener "listener${i}_${j}" $topic $resultdir/listener$i\_$j\_rate$rate\_size$size.csv
      done
    done
    wait 60
    stop_processes
    wait 30
    stop_cpu_dump
    echo ">>>> Done with run ${name} <<<<"
}

exit_function () {
    trap SIGINT              # Restore signal handling for SIGINT
    echo "Interrupted...kill all processes"
    killall dump_cpu_load.sh talker.py listener.py roscore python
    exit                     #   then exit script.
}

# --- Main script body

trap "exit_function" INT            # Set up SIGINT trap to call function.

n_talkers_m_listeners 1 1 50 1000000 # Easy test: 1 talker, 1 listener, sending 1MB msg at 50Hz
wait 10
realistic_test # Realistic test: 1 talker (1MB at 50Hz) + two talkers (1KB at 50Hz) + a few listeners (1+2+1)
wait 10
n_talkers_m_listeners 5 2 100 50000  # Stress test: 5 talkers each 5 listeners, sending 1MB msgs at 100 Hz

echo ">> Evaluation done"

trap SIGINT
