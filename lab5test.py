import random
import time
import argparse
import sys

print(sys.argv)
time_run = float(input("Time to run for (in seconds):"))

start_time = time.time()
'''run_time = int(sys.argv[1])'''
itime = start_time

while itime < (start_time + time_run):
    itime = time.time()
    idata = random.random()
    print(itime, idata)
    time.sleep(1)