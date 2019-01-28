#!/usr/bin/python
import rosbag
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
sys.path.append("utils")
import figure_utils
import rosbag_utils
import math
import copy
import pickle as pickle
from matplotlib.pyplot import step, show
from collections import deque
from mrs_msgs.msg import ProfilerUpdate

import json
import yaml
sys.path.append("utils")

endl = "\n"

from json import encoder
encoder.FLOAT_REPR = lambda o: format(o, '.5f')

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   y["expected_start"] = float(msg.expected_start)
   y["real_start"] = float(msg.real_start)
   y["duration"] = float(msg.duration)
   # print("y: {}".format(y))
   return json.dumps(y, indent=4)

# #{ loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0):

def loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0, out_file_name = "out.txt"):

    # #{ postProcessData(list_of_msgs, start_time, end_time)
    
    def postProcessData(list_of_msgs, start_time, end_time):
    
        print("Post-processing messages:")
        print("	start_time", start_time)
        print("	end_time", end_time)
    
        # get all ProfilerUpdate-type messaages from the bag
        profiler_msgs = []
    
        for msg in list_of_msgs:
    
            topic_time = rosbag_utils.getStampTimeSec(msg.stamp)
    
            # filter only message in the certain time range
            if topic_time > start_time and topic_time < end_time:
    
                profiler_msgs.append(msg)
    
                if topic_time >= end_time:
                    break
    
        return profiler_msgs
    
    # #} end of postProcessData()

    try:
        print("opening "+ bag_filename)
        bag = rosbag.Bag(bag_filename)
    except rosbag.bag.ROSBagUnindexedException:
        print("error while opening the rosbag")
        sys.exit()

    print("done opening the rosbag")

    time_start = bag.get_start_time()
    time_end = bag.get_end_time()

    if required_time_end == 0:
        required_time_end = time_end-time_start

    profiler_data = rosbag_utils.loadProfilerTopics(bag, uav_name, time_start, time_end)

    print("bag file closed")

    data = postProcessData(profiler_data, time_start+required_time_start, time_start+required_time_end)

    return [data, required_time_start, required_time_end]

# #} end of loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0):

def exportData(profiler_data, required_time_start, required_time_end):

    # find unique names of topics in profiler
    node_names = {"%s: %s" % (msg.node_name, msg.routine_name) for msg in profiler_data}
    node_names = sorted(node_names);

    data = [[float(i)] for i in range(len(node_names))]
    times = [[required_time_start] for i in range(len(node_names))]
    errors_data= []
    errors_times= []

    # generate line plots
    with open("/home/klaxalk/"+out_file_name, "w") as outfile:

        outfile.write("["+endl)

        for msg in profiler_data:

            print("exporting message from t={}".format(msg.stamp.secs))

            # msg.event
            temp = msg2json(msg)
            outfile.write(temp)

            if msg != profiler_data[-1]:
                outfile.write(","+endl)
            else:
                outfile.write(endl)

        outfile.write("]")

if __name__ == "__main__":

    # bag_file = "/home/klaxalk/gain_tuning_2.bag"
    # uav_name = "uav10"
    # start_time = 80
    # stop_time = 400
    # out_file_name = "gain_tuning_2.txt"

    bag_file = "/home/klaxalk/realsense_orb_slam_all_cameras.bag"
    uav_name = "uav10"
    start_time = 0
    stop_time = 0
    out_file_name = "orb_slam.txt"

    [data, start_time, end_time] = loadProfilerDataFromRosbag(bag_file, uav_name, start_time, stop_time, out_file_name)

    exportData(data, start_time, end_time)
