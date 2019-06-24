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

# step in the graphs that shows when the profiler was active
visualization_step = 0.8

# #{ loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0):

def loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0):

    # #{ process_data(list_of_msgs, start_time, end_time):

    def process_data(list_of_msgs, start_time, end_time):
        print("processing_data")
        print("start_time", start_time)
        print("end_time", end_time)

        msg_index = 0
        profiler_msgs = [ ProfilerUpdate() for _ in range(len(list_of_msgs))]

        for msg in list_of_msgs:
            topic_time = rosbag_utils.getStampTimeSec(msg.stamp)
            if topic_time > start_time and topic_time < end_time:
                profiler_msgs[msg_index] = msg
                msg_index += 1

                if topic_time >= end_time:
                    break

        data = profiler_msgs[0:(msg_index)]
        print("readed topics:", msg_index)
        return data

    # #} end of

    # ----------------------------------------
    cache_loaded = False
    cache_filename = '/tmp/cache_profiler.pkl'

    if os.path.isfile(cache_filename):
        try:
            with open(cache_filename, 'rb') as cache_file:
                cached = pickle.load(cache_file)
                if  cached[0] == bag_filename:
                    profiler_data = cached[1]
                    cache_loaded = True
                    print("Loaded data from cached file "+ cache_filename)
                else:
                    print("Cached data differs from current options, skipping")
        except Exception as e:
            # everything else, possibly fatal
            print("can not parse cache file "+cache_filename)
            print(e)
    else:
        print("No cache found, skipping cache loading")

    if not cache_loaded:
        try:
            print("opening "+ bag_filename)
            bag = rosbag.Bag(bag_filename)

        except rosbag.bag.ROSBagUnindexedException:
            print("rosbag is not indexed")
            sys.exit()

        print("done opening")

        #loading uav positions
        time_start = bag.get_start_time()
        time_end = bag.get_end_time()

        profiler_data = rosbag_utils.loadProfilerTopics(bag, uav_name, time_start, time_end)

        print("bag file closed")

        with open(cache_filename, 'wb') as output:  # Overwrites any existing file.
            cache = [bag_filename, profiler_data]
            pickle.dump(cache, output, pickle.HIGHEST_PROTOCOL)
            print("cache file saved")

    if required_time_end == 0:
        required_time_end = time_end

    if required_time_start == 0:
        required_time_start = time_start

    data = process_data(profiler_data, required_time_start, required_time_end)

    return [data, required_time_start, required_time_end]

# #} end of loadProfilerDataFromRosbag(bag_filename, uav_name, required_time_start = 0, required_time_end = 0):

# #{ plot_all_data_separately(profiler_data, required_time_start, required_time_end):

def plot_all_data_separately(profiler_data, required_time_start, required_time_end):
    # create figure
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # set fullscreen
    figManager = plt.get_current_fig_manager()
    figManager.full_screen_toggle()

    # find unique names of topics in profiler
    node_names = {"%s: %s" % (msg.node_name, msg.routine_name) for msg in profiler_data}
    node_names = sorted(node_names);

    data = [[float(i)] for i in range(len(node_names))]
    times = [[required_time_start] for i in range(len(node_names))]
    errors_data= []
    errors_times= []

    # generate line plots
    for msg in profiler_data:
        name = "%s: %s" % (msg.node_name, msg.routine_name)
        index = node_names.index(name)

        end_time = rosbag_utils.getStampTimeSec(msg.stamp)

        if msg.event == msg.END:

            if msg.real_start == 0:
                print("start time 0")
                continue

            times[index].append(msg.real_start)
            data[index].append(index + visualization_step)

            times[index].append(end_time)
            data[index].append(index)

            if (msg.is_periodic):

                expected_start = msg.expected_start
                real_start = msg.real_start
                if (real_start - expected_start) > 0.0010:
                    errors_data.append(index)
                    errors_times.append(expected_start)
                    print("real_start, expected_start: {}, {}".format(real_start, expected_start))


    # fill the graphs with last value until end_time
    for index in range(len(node_names)):
        data[index].append(data[index][-1])
        times[index].append(required_time_end)

    # plot data
    for index in range(len(node_names)):
        ax.step(times[index], data[index], where='post', color='blue')

    # plot delayed starts
    for index in range(len(errors_data)):
        ax.plot([errors_times[index], errors_times[index]],[errors_data[index], errors_data[index] + visualization_step], color='red')

    # add labeling of y-axis with step one
    ax.set_yticks([i+0.5 for i in range(len(node_names))])
    ax.set_yticklabels(node_names)

    # adjust size so labels are visible
    plt.subplots_adjust(left=0.2, right=0.9, top=0.9, bottom=0.1)

    # add labels for axes
    ax.set_xlabel('time [s]')
    ax.set_ylabel('profiler topic name [-]')
    
    plt.show()

# #} end of plot_all_data_separately(profiler_data, required_time_start, required_time_end):

# #{ plot_sheduled_data(profiler_data, required_time_start, required_time_end):

def plot_sheduled_data(profiler_data, required_time_start, required_time_end):

    # #{ sort_by_state_and_name(data_list):

    def sort_by_state_and_name(data_list):
        n_data = len(data_list)
        # print(n_data)
        # print("=======================")
        # print(data_list)
        new_list = deque()
        is_done = [False for i in range(len(data_list))]
        for i in range(n_data):
            if is_done[i]:
                continue

            data_1 = data_list[i]
            for j in range(n_data):
                if is_done[j]:
                    continue

                data_2 = data_list[j]
                if i != j:
                    data_1_name = {"%s: %s" % (data_1.node_name, data_1.routine_name)}
                    data_2_name = {"%s: %s" % (data_2.node_name, data_2.routine_name)}
                    if data_1_name == data_2_name:

                        new_list.append(data_1)
                        is_done[i] = True

                        new_list.append(data_2)
                        is_done[j] = True

                        break

        for i in range(n_data):
            if is_done[i]:
                continue
            if data_list[i].event == ProfilerUpdate.START:
                new_list.append(data_list[i])
            else:
                new_list.appendleft(data_list[i])

        # print("---------------------")
        # print(new_list)
        # print("---------------------")
        return new_list

    # #} 

    # create figure
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # set fullscreen
    figManager = plt.get_current_fig_manager()
    figManager.full_screen_toggle()

    # find unique names of topics in profiler
    node_names = {"%s: %s" % (msg.node_name, msg.routine_name) for msg in profiler_data}
    node_names = sorted(node_names);

    number_of_threads = 1
    max_number_of_threads = 64

    # create variables
    states = [False for _ in range(number_of_threads)]
    data = [[float(i)] for i in range(number_of_threads)]
    times = [[required_time_start] for i in range(number_of_threads)]

    position_in_core = [int(-1) for i in range(len(node_names))]
    active = [False for i in range(len(node_names))]

    list_of_message_in_the_same_time = []
    previous_msg_time = 0

    for msg_data in profiler_data:
        time = rosbag_utils.getStampTimeSec(msg_data.stamp)
        if previous_msg_time != time:

            # processed data
            list_of_message_in_the_same_time = sort_by_state_and_name(list_of_message_in_the_same_time)
            # print(list_of_message_in_the_same_time)
            for msg in list_of_message_in_the_same_time:
                name = "%s: %s" % (msg.node_name, msg.routine_name)
                index = node_names.index(name)
                time = rosbag_utils.getStampTimeSec(msg.stamp)
                position = position_in_core[index]

                if msg.event == msg.START:

                    if active[index]:
                        print("routine {} is already active, time: {}".format(name, time))
                        # data[position].append(position)
                        states[position] = False
                        active[index] = False

                    thread_index = -1
                    for i in range(number_of_threads):
                        if not states[i]:
                            thread_index = i
                            states[i] = True
                            break;

                    if thread_index == -1:
                        number_of_threads += 1
                        if number_of_threads > max_number_of_threads:
                            print("Reached maximal number of threads that is: {}".format(max_number_of_threads))
                            sys.exit()

                        print("Increasing number of threads to: {}".format(number_of_threads))
                        thread_index = number_of_threads - 1
                        data.append([thread_index])
                        times.append([required_time_start])
                        states.append(True)

                    position = thread_index
                    position_in_core[index] = position
                    active[index] = True
                    data[position].append(position + visualization_step)
                else:
                    if position < 0:
                        continue
                    data[position].append(position)
                    states[position] = False
                    active[index] = False

                times[position].append(time)

            list_of_message_in_the_same_time.clear()
            list_of_message_in_the_same_time.append(msg_data)
        else:
            list_of_message_in_the_same_time.append(msg_data)

        previous_msg_time = time


    # fill the graphs with last value until end_time
    for index in range(number_of_threads):
        data[index].append(data[index][-1])
        times[index].append(required_time_end)

    # plot data
    for index in range(number_of_threads):
        ax.step(times[index], data[index], where='post', color='blue')

    # add labeling of y-axis with step one
    ax.set_yticks([i + 0.5 for i in range(number_of_threads)])
    ax.set_yticklabels([i for i in range(number_of_threads)])
    
    # add labels for axes
    ax.set_xlabel('time [s]')
    ax.set_ylabel('thread number [-]')

    plt.show()

# #} end of plot_sheduled_data(profiler_data, required_time_start, required_time_end):

if __name__ == "__main__":

    bag_file = "/home/klaxalk/22_2019_06_21_12_22_14/_2019-06-21-12-22-26.bag"
    uav_name = "uav5"
    start_time = 1561112697-1.5
    stop_time = 1561112697+1.5

    [data, start_time, end_time] = loadProfilerDataFromRosbag(bag_file, uav_name, start_time, stop_time)
    plot_all_data_separately(data, start_time, end_time)
    # plot_sheduled_data(data, start_time, end_time)
