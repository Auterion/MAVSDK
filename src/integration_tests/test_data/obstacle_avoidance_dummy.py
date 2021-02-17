#!/usr/bin/env python3
import multiprocessing
import os
import sys

from functools import partial
from signal import pause, signal, SIGINT, SIGTERM
from threading import Thread
from time import sleep

"""
Dummy obstacle avoidance script
"""

cycle_counter = 0
stop_signal = False

def run():
    global cycle_counter
    while not stop_signal:
        cycle_counter += 1
        sleep(0.1)

def signal_handler(th, sig, frame):
    global stop_signal
    stop_signal = True

def main():
    th = Thread(target=run, args=())
    th.start()

    PID = str(os.getpid())
    with open('obs_avoid_service.pid', 'w') as file:
        file.write(PID)

    signal(SIGINT, partial(signal_handler, th))
    signal(SIGTERM, partial(signal_handler, th))

    print("\nObstacle avoidance service started!\n");

    pause()

    print("\nObstacle avoidance service stopped! Ran for {} cycles.".format(cycle_counter));
    th.join()

if __name__ == '__main__':
    main()
