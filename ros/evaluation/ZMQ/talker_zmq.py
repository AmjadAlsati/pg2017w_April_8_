import sys
import time
import zmq
import struct 
import argparse


def talker(size, name, port, rate=50):
    context = zmq.Context()
    sock = context.socket(zmq.PUB)
    sock.bind("tcp://*:"+port)

    while True:
        time.sleep(1.0/rate)
        now = time.time() * 1000 * 1000
#        msg=str(now)+(max(size - 18, 0)*'\0')
#        print("Publish "+msg)
#        sock.send_string(msg)
        sock.send(struct.pack('!d', time.time()) + ('\0' * min(size - 8, 0)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Simple publisher, sends timestamps followed by a configurable padding.")
    parser.add_argument('-r', '--rate', type=int, default=50, help="The rate at which messages are generated.")
    parser.add_argument('size', type=int, help="Message length in byte (at least 8).")
    parser.add_argument('-n', '--name', type=str, default="talker", help="Name of the talker.")
    parser.add_argument('-p', '--port', type=str, default="8080", help="Port number.")

    args = parser.parse_args()
    talker(args.size, args.name, args.port, args.rate)
