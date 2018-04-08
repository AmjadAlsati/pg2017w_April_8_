import struct
import sys
import zmq
import time
import argparse
import os

runs = 0

def listener(port, name):
    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, '')  
    p="tcp://localhost:"+port
#    print("Port: ",p)
    sock.connect(p)

    while True:
        msg= sock.recv()
        now=time.time()*1000*1000
        print (msg)
        (timestamp,) = struct.unpack('!d', msg[:8])
#        timestamp = float(message[:18])
        delay=now-timestamp
#        print ("timestamp: ",timestamp," now: ",now," delay:",now-timestamp,"ns")

        global runs
        runs += 1
        with open(outputpath, "a") as resultfile:
            resultfile.write("{},{},{},{},{}\n".format(os.getpid(),runs,timestamp,now,delay))

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Simple subscriber, receives messages and evaluated duration between sending and receiving")
    parser.add_argument('-o', '--outputpath', type=str, default="results.csv", help="Path to output file (should be a .csv file).")
    parser.add_argument('-p', '--port', type=str, default="8080", help="Port number.")
    parser.add_argument('-n', '--name', type=str, default="listener", help="Name of the listener.")

    args = parser.parse_args()
    outputpath = args.outputpath
    with open(outputpath, "w") as resultfile:
        resultfile.write("ProcessID,Run,TimeSent,TimeRcvd,Duration\n")
    listener(args.port, args.name)
