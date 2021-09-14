import rosbag
import os
import sys
import random
import numpy as np
import copy
import rospy
import cv2
import tf_conversions
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mrs_msgs.msg import ProfilerUpdate
# from itertools import zip, repeat


def getStampTimeSec(timeStamp):
    return float(str(timeStamp)) / 1000000000.0

def syncStartEndTimes(messageList):
    """
    expect messageList to contain list of numpy matrices where forst column is time
    """
    maxStartTime = messageList[0][0, 0]
    minEndTime = messageList[0][0, 0]
    for i in range(len(messageList)):
        numRecords = messageList[i].shape[0]
        if messageList[i][0, 0] > maxStartTime:
            maxStartTime = messageList[i][0, 0]

        if messageList[i][numRecords - 1, 0] < minEndTime:
            minEndTime = messageList[i][numRecords - 1, 0]

    startIndexes = [0] * len(messageList)
    endIndexes = [0] * len(messageList)
    for i in range(len(messageList)):
        pass

def getClosestTimeIndexTo(matrix, time):
    print("getClosestTimeIndexTo")
    print("time", time)
    closestTime = time
    closestIndex = 0
    closestTimeDifference = sys.float_info.max
    for rowid in range(matrix.shape[0]):
        diff = abs(matrix[rowid, 0] - time)
        if(diff < closestTimeDifference):
            closestTime = matrix[rowid, 0]
            closestIndex = rowid
            closestTimeDifference = diff
    print("closestTimeDifference ", closestTimeDifference)
    print("closestIndex ", closestIndex)
    return closestIndex

def loadOdometryWhole(bag, uav_name, odom_topic="/mrs_odometry/new_odom"):
    print("loadOdometryWhole")
    odom_topic = "/" + uav_name + odom_topic
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [odom_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))

    positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            positions[positions_num, 0] = (float(str(msg.header.stamp)) / 1000000000.0)
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = msg.pose.pose.position.z
            positions_num += 1

        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    print("readed topics:", readedTopicsNums)
    return positions


def loadOdometryInTracking(bag, uav_name, odom_topic="/mrs_odometry/new_odom"):

    print("loadOdometryInTracking")
    odom_topic = "/" + uav_name + odom_topic
    mpc_diagnostics_state_topic = "/" + uav_name + "/trackers_manager/mpc_tracker/diagnostics"
    topics_to_read = [odom_topic, mpc_diagnostics_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))
    tracking = False
    positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and tracking:
            positions[positions_num, 0] = (float(str(msg.header.stamp)) / 1000000000.0)
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = msg.pose.pose.position.z
            positions_num += 1


        if topic == mpc_diagnostics_state_topic:
            if (not tracking) and msg.tracking_trajectory:
                print("uav ", uav_name, " tracking_trajectory ", msg.tracking_trajectory)
                positions_num = 0
            tracking = msg.tracking_trajectory


        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    print("readed topics:", readedTopicsNums)
    return positions


def loadOdometryAfterArming(bag, uav_name, odom_topic="/mrs_odometry/new_odom"):
    print("loadOdometryAfterArming")
    odom_topic = "/" + uav_name + odom_topic
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [odom_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))
    armed = False
    positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            positions[positions_num, 0] = (float(str(msg.header.stamp)) / 1000000000.0)
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = msg.pose.pose.position.z
            positions_num += 1


        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                armed = msg.armed

        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    print("readed topics:", readedTopicsNums)
    return positions

def loadOdometryAfterArmingWithRTKz(bag, uav_name, odom_topic="/mrs_odometry/new_odom"):
    print("loadOdometryAfterArmingWithRTKz")
    odom_topic = "/" + uav_name + odom_topic
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    rtk_local_topic = "/" + uav_name + "/rtk_gps/local"
    topics_to_read = [odom_topic, mavros_state_topic, rtk_local_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))
    last_rtk_pos = np.zeros(shape=(1, 4))
    armed = False
    positions_num = 0
    armint_z = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            positions[positions_num, 0] = (float(str(msg.header.stamp)) / 1000000000.0)
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = last_rtk_pos[0, 3] - armint_z
            positions_num += 1

        if topic == rtk_local_topic:
            last_rtk_pos[0, 0] = getStampTimeSec(msg.header.stamp)
            last_rtk_pos[0, 1] = msg.position.x
            last_rtk_pos[0, 2] = msg.position.y
            last_rtk_pos[0, 3] = msg.position.z

        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                armed = msg.armed
                if armed and armint_z == 0:
                    armint_z = last_rtk_pos[0, 3]

        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    print("readed topics:", readedTopicsNums)
    return positions

def getStartTimeOfTracking(bag, uav_name, num):
    print("getStartTimeOfTracking")
    mpc_diagnostics_state_topic = "/" + uav_name + "/trackers_manager/mpc_tracker/diagnostics"
    topics_to_read = [mpc_diagnostics_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    counter = 0;
    tracking = False;
    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == mpc_diagnostics_state_topic:
            if (not tracking) and msg.tracking_trajectory:
                print("uav ", uav_name, " tracking_trajectory ", msg.tracking_trajectory)
                counter += 1
            tracking = msg.tracking_trajectory
            if counter == num:
                return (int(str(msg.header.stamp)) / 1000000000.0)

    return 0

def loadOdometryAfterArmingEntTime(bag, uav_name, end_time_relative, odom_topic="/mrs_odometry/new_odom"):
    print("loadOdometryAfterArmingEntTime")
    odom_topic = "/" + uav_name + odom_topic
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [odom_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))
    armed = False
    positions_num = 0
    end_in = sys.float_info.max

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            topic_time = getStampTimeSec(msg.header.stamp)
            if topic_time >= end_in:
                print("ending after end_time_relative", end_time_relative)
                break
            positions[positions_num, 0] = topic_time
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = msg.pose.pose.position.z
            positions_num += 1


        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                end_in = end_time_relative + getStampTimeSec(msg.header.stamp)
                armed = msg.armed

        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    print("readed topics:", readedTopicsNums)
    return positions


def loadOdometryBetweenArming(bag, uav_name):
    print("loadOdometryBetweenArming")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [odom_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    position_between = []
    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 7))
    armed = False
    positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            positions[positions_num, 0] = (int(str(msg.header.stamp)) / 1000000000.0)
            positions[positions_num, 1] = msg.pose.pose.position.x
            positions[positions_num, 2] = msg.pose.pose.position.y
            positions[positions_num, 3] = msg.pose.pose.position.z
            explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(explicit_quat)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            positions[positions_num, 4] = yaw
            positions[positions_num, 5] = pitch
            positions[positions_num, 6] = roll



            positions_num += 1


        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                armed = msg.armed
                if not armed:
                    print("add part ", len(position_between), "with positions_num", positions_num)
                    part_between = positions[0:(positions_num), :]
                    position_between.append(part_between)
                    positions_num = 0

        readedTopicsNums[topic] += 1

    print("positions_num", positions_num)
    for pos_part_i in range(len(position_between)):
        print("size pos_part", pos_part_i, "has len", position_between[pos_part_i].shape[0])
    print("readed topics:", readedTopicsNums)
    return position_between


def loadOdometryBetweenTime(bag, uav_name, start_time, end_time):
    print("loadOdometryBetweenTime")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [odom_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))

    positions = np.zeros(shape=(bag.get_message_count(odom_topic), 4))
    positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            if topic_time > start_time and topic_time < end_time:
                positions[positions_num, 0] = topic_time
                positions[positions_num, 1] = msg.pose.pose.position.x
                positions[positions_num, 2] = msg.pose.pose.position.y
                positions[positions_num, 3] = msg.pose.pose.position.z
                positions_num += 1

            if topic_time >= end_time:
                break



    print("positions_num", positions_num)
    positions = positions[0:(positions_num), :]
    return positions

def loadOdometrySetpointAttitudeBetweenArming(bag, uav_name):
    print("loadOdometrySetpointAttitudeBetweenArming")
    attitude_topic = "/" + uav_name + "/mavros/setpoint_raw/attitude"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [attitude_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))

    setpoints_between = []
    setpoints = np.zeros(shape=(bag.get_message_count(attitude_topic), 4))
    setpoint_num = 0
    armed = False

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == attitude_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            if armed and topic_time > 0:
                setpoints[setpoint_num, 0] = topic_time
                explicit_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                euler = tf.transformations.euler_from_quaternion(explicit_quat)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]
                setpoints[setpoint_num, 1] = yaw
                setpoints[setpoint_num, 2] = pitch
                setpoints[setpoint_num, 3] = roll
                setpoint_num += 1

        if topic == mavros_state_topic:
            # print(mavros_state_topic)
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                armed = msg.armed
                if not armed:
                    # print("add to setpoints_between")
                    part_between = setpoints[0:(setpoint_num), :]
                    setpoints_between.append(part_between)
                    setpoint_num = 0

    for setpoint_part_i in range(len(setpoints_between)):
        print("size setpoint_part", setpoint_part_i, "has len", setpoints_between[setpoint_part_i].shape[0])

    print("setpoint_num", setpoint_num)
    return setpoints_between

def loadOdometrySetpointBetweenArming(bag, uav_name):
    print("loadOdometrySetpointBetweenArming")
    setpoint_topic = "/" + uav_name + "/trackers_manager/mpc_tracker/cmd_pose"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [setpoint_topic, mavros_state_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))

    setpoints_between = []
    setpoints = np.zeros(shape=(bag.get_message_count(setpoint_topic), 7))
    setpoint_num = 0
    armed = False

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == setpoint_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            if armed:
                setpoints[setpoint_num, 0] = topic_time
                setpoints[setpoint_num, 1] = msg.pose.pose.position.x
                setpoints[setpoint_num, 2] = msg.pose.pose.position.y
                setpoints[setpoint_num, 3] = msg.pose.pose.position.z
                explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
                euler = tf.transformations.euler_from_quaternion(explicit_quat)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]
                setpoints[setpoint_num, 4] = yaw
                setpoints[setpoint_num, 5] = pitch
                setpoints[setpoint_num, 6] = roll
                setpoint_num += 1

        if topic == mavros_state_topic:
            # print(mavros_state_topic)
            if (not armed) == msg.armed:
                print("uav ", uav_name, " armed ", msg.armed)
                armed = msg.armed
                if not armed:
                    # print("add to setpoints_between")
                    part_between = setpoints[0:(setpoint_num), :]
                    setpoints_between.append(part_between)
                    setpoint_num = 0



    print("setpoint_num", setpoint_num)
    return setpoints_between

def loadOdometrySetpointBetweenTime(bag, uav_name, start_time, end_time):
    print("loadOdometrySetpointBetweenTime")
    setpoint_topic = "/" + uav_name + "/trackers_manager/mpc_tracker/cmd_pose"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    topics_to_read = [setpoint_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))

    setpoints = np.zeros(shape=(bag.get_message_count(setpoint_topic), 4))
    setpoint_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == setpoint_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            if topic_time > start_time and topic_time < end_time:
                setpoints[setpoint_num, 0] = topic_time
                setpoints[setpoint_num, 1] = msg.pose.pose.position.x
                setpoints[setpoint_num, 2] = msg.pose.pose.position.y
                setpoints[setpoint_num, 3] = msg.pose.pose.position.z
                setpoint_num += 1

            if topic_time >= end_time:
                break



    print("setpoint_num", setpoint_num)
    positions = setpoints[0:(setpoint_num), :]
    return positions


def loadPositionZeroThrustInFlight(bag, uav_name):
    print("loadPositionZeroThrustInFlight")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    mavros_attitude = "/" + uav_name + "/mavros/setpoint_raw/target_attitude"
    topics_to_read = [odom_topic, mavros_state_topic, mavros_attitude]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    positions_zero_trust = []

    armed = False

    last_position = np.zeros(shape=(1, 4))
    last_trust = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            last_position[0, 0] = (int(str(msg.header.stamp)) / 1000000000.0)
            last_position[0, 1] = msg.pose.pose.position.x
            last_position[0, 2] = msg.pose.pose.position.y
            last_position[0, 3] = msg.pose.pose.position.z


        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                print("uav ",uav_name," armed ",msg.armed)
                armed = msg.armed
                return (int(str(msg.header.stamp)) / 1000000000.0)

        if topic == mavros_attitude:
            if msg.thrust == 0 and last_trust > 0 and armed:
                positions_zero_trust.append(copy.deepcopy(last_position))
            last_trust = msg.thrust

        readedTopicsNums[topic] += 1

    print("readed topics:", readedTopicsNums)
    return positions_zero_trust

def loadPositionsLandingStates(bag, uav_name):
    print("loadPositionsLandingStates")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    landing_cross_feedback = "/" + uav_name + "/landing_cross_controller/feedback"
    topics_to_read = [odom_topic, mavros_state_topic, landing_cross_feedback]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)
    num_landing_feedbacks = bag.get_message_count(landing_cross_feedback)
    positions_zero_trust = []

    armed = False

    feedback_changes = []
    last_position = np.zeros(shape=(1, 4))


    actual_state_id = -1
    actual_state_start = 0
    actual_state_message = ""

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic and armed:
            last_position[0, 0] = (int(str(msg.header.stamp)) / 1000000000.0)
            last_position[0, 1] = msg.pose.pose.position.x
            last_position[0, 2] = msg.pose.pose.position.y
            last_position[0, 3] = msg.pose.pose.position.z


        if topic == mavros_state_topic:
            if (not armed) == msg.armed:
                # print("uav ",uav_name," armed ",msg.armed)
                armed = msg.armed

        if topic == landing_cross_feedback:

            if(actual_state_id > 0) :
                feedback_changes.append([actual_state_start, int(str(msg.header.stamp)) / 1000000000.0, actual_state_id, actual_state_message]);

            actual_state_id = msg.feedback.stage
            actual_state_start = int(str(msg.header.stamp)) / 1000000000.0
            actual_state_message = msg.feedback.message

            print(msg.feedback.message)

        readedTopicsNums[topic] += 1

    if(actual_state_id > 0) :
        feedback_changes.append([actual_state_start, last_position[0, 0], actual_state_id, actual_state_message]);

    print("readed topics:", readedTopicsNums)
    return feedback_changes


def loadCrossPositions(bag, uav_name):
    print("loadCrossPositions")
    cross_position_topic = "/" + uav_name + "/cross_detector/cross_position"
    topics_to_read = [cross_position_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    cross_positions = np.zeros(shape=(bag.get_message_count(cross_position_topic), 4))

    cross_positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == cross_position_topic :
            cross_positions[cross_positions_num, 0] = (int(str(msg.header.stamp)) / 1000000000.0)
            cross_positions[cross_positions_num, 1] = msg.pose.position.x
            cross_positions[cross_positions_num, 2] = msg.pose.position.y
            cross_positions[cross_positions_num, 3] = msg.pose.position.z
            cross_positions_num += 1

        readedTopicsNums[topic] += 1

    print("cross_positions_num", cross_positions_num)
    cross_positions = cross_positions[0:(cross_positions_num), :]
    print("readed topics:", readedTopicsNums)
    return cross_positions

def loadCrossPositionsEstimator(bag, uav_name):
    print("loadCrossPositions")
    cross_estimate_topic = "/" + uav_name + "/landing_cross_estimator/cross_odometry"
    topics_to_read = [cross_estimate_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    cross_positions = np.zeros(shape=(bag.get_message_count(cross_estimate_topic), 4))

    cross_positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == cross_estimate_topic :
            cross_positions[cross_positions_num, 0] = (int(str(msg.header.stamp)) / 1000000000.0)
            cross_positions[cross_positions_num, 1] = msg.pose.pose.position.x
            cross_positions[cross_positions_num, 2] = msg.pose.pose.position.y
            cross_positions[cross_positions_num, 3] = 1.85
            cross_positions_num += 1

        readedTopicsNums[topic] += 1

    print("cross_positions_num", cross_positions_num)
    cross_positions = cross_positions[0:(cross_positions_num), :]
    print("readed topics:", readedTopicsNums)
    return cross_positions


def loadGraspingStatesBetweenTime(bag, uav_name, start_time, end_time):
    print("loadGraspingStatesBetweenTime")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    grasping_feedback = "/" + uav_name + "/landing_object_controller/feedback"
    topics_to_read = [odom_topic, grasping_feedback]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)
    num_landing_feedbacks = bag.get_message_count(grasping_feedback)
    positions_zero_trust = []



    feedback_changes = []
    last_position = np.zeros(shape=(1, 4))


    actual_state_id = -1
    actual_state_start = 0
    actual_state_message = ""
    addedLastChange = False
    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            if topic_time > start_time and topic_time < end_time:
                last_position[0, 0] = topic_time
                last_position[0, 1] = msg.pose.pose.position.x
                last_position[0, 2] = msg.pose.pose.position.y
                last_position[0, 3] = msg.pose.pose.position.z
            if topic_time >= end_time:
                break

        if topic == grasping_feedback:

            if actual_state_id == -1:
                actual_state_id = msg.feedback.stage
                actual_state_start = getStampTimeSec(msg.header.stamp)
                actual_state_message = msg.feedback.message

            if(actual_state_id != msg.feedback.stage) :
                message_time = getStampTimeSec(msg.header.stamp)
                if message_time > start_time and message_time < end_time:
                    print("state change from", actual_state_message, "to", msg.feedback.message, "at", message_time)
                    print("save change from", actual_state_start, "to", message_time, actual_state_message)
                    feedback_changes.append([actual_state_start, message_time, actual_state_id, actual_state_message]);
                if message_time > end_time and not addedLastChange:
                    addedLastChange = True
                    print("state change from", actual_state_message, "to", msg.feedback.message, "at", message_time)
                    print("save change from", actual_state_start, "to", end_time, actual_state_message)
                    feedback_changes.append([actual_state_start, end_time, actual_state_id, actual_state_message]);


                actual_state_id = msg.feedback.stage
                actual_state_start = message_time
                actual_state_message = msg.feedback.message

            # print(msg.feedback.message)

        readedTopicsNums[topic] += 1

    if(actual_state_id >= 0 and not addedLastChange) :
        print("add last state ", actual_state_message, "from", actual_state_start, "to", last_position[0, 0])
        feedback_changes.append([actual_state_start, last_position[0, 0], actual_state_id, actual_state_message]);

    print("readed topics:", readedTopicsNums)
    return feedback_changes

def loadUAVPositionOfGraspsWithRTKz(bag, uav_name):
    print("loadUAVPositionOfGraspsWithRTKz")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    mavros_state_topic = "/" + uav_name + "/mavros/state"
    grasping_feedback = "/" + uav_name + "/landing_object_controller/feedback"
    grasping_object_current_target = "/" + uav_name + "/mbzirc_detector/object_array"
    rtk_local_topic = "/" + uav_name + "/rtk_gps/local"
    topics_to_read = [odom_topic, mavros_state_topic, grasping_feedback, grasping_object_current_target, rtk_local_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)
    num_landing_feedbacks = bag.get_message_count(grasping_feedback)

    IDLE_STATE = 0
    ALIGN_STATE = 1
    LAND_STATE = 2
    WAIT_STATE = 3
    ALIGN2_STATE = 4
    GRASP_STATE = 5
    REPEAT_STATE = 6
    TAKEOFF_STATE = 7
    ABOART_STATE = 8
    ALIGN_DROP_STATE = 9
    LAND_TO_DROP_STATE = 10
    DROPPING_STATE = 11
    WAITING_AFTER_DROPPING = 12
    WAIT_1_FAIL = 13
    PREEMPTED_STATE = 14

    OBJECT_LONG_MIDDLE = 0
    OBJECT_RED = 1
    OBJECT_GREEN = 2
    OBJECT_BLUE = 3
    OBJECT_MOVING = 5
    OBJECT_STATIC = 999
    OBJECT_LONG_END = 1000

    feedback_changes = []
    last_position = np.zeros(shape=(1, 4))

    last_grasp_position = np.zeros(shape=(1, 5))

    grasp_positions = np.zeros(shape=(bag.get_message_count(grasping_feedback), 5))
    last_grasping_target_type = 0
    last_rtk_pos = np.zeros(shape=(1, 4))
    armint_z = 0
    actual_state_id = -1
    actual_state_start = 0
    actual_state_message = ""
    grasp_num = 0
    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            last_position[0, 0] = topic_time
            last_position[0, 1] = msg.pose.pose.position.x
            last_position[0, 2] = msg.pose.pose.position.y
            last_position[0, 3] = last_rtk_pos[0, 3] - armint_z

        if topic == grasping_object_current_target:
            # if len(msg.objects) > 0:
            #    typessrt = ""
            #    ifor i in range(len(msg.objects)):
            #        typessrt += str(msg.objects[i].type) + " "
            #    print("types:",typessrt)
            if len(msg.objects) == 1:
                last_grasping_target_type = msg.objects[0].type
                # print("type",msg.objects[0].type)
        if topic == rtk_local_topic:
            last_rtk_pos[0, 0] = getStampTimeSec(msg.header.stamp)
            last_rtk_pos[0, 1] = msg.position.x
            last_rtk_pos[0, 2] = msg.position.y
            last_rtk_pos[0, 3] = msg.position.z


        if topic == grasping_feedback:

            if actual_state_id == -1:
                actual_state_id = msg.feedback.stage
                actual_state_start = getStampTimeSec(msg.header.stamp)
                actual_state_message = msg.feedback.message

            if(actual_state_id != msg.feedback.stage) :
                if(actual_state_id >= 0 and (actual_state_id != ALIGN2_STATE and msg.feedback.stage == ALIGN2_STATE)):
                    print("change to ALIGN2_STATE with last grasping target ", last_grasping_target_type)

                if(actual_state_id >= 0 and (actual_state_id == GRASP_STATE and msg.feedback.stage == TAKEOFF_STATE)):
                    # have grasped object but not sure about succes
                    print("state change from", actual_state_message, "to", msg.feedback.message, "object type ", last_grasping_target_type)
                    last_grasp_position[0, 0] = last_position[0, 0]
                    last_grasp_position[0, 1] = last_position[0, 1]
                    last_grasp_position[0, 2] = last_position[0, 2]
                    last_grasp_position[0, 3] = last_position[0, 3]
                    last_grasp_position[0, 4] = last_grasping_target_type

                if(actual_state_id >= 0 and (actual_state_id == TAKEOFF_STATE and msg.feedback.stage == IDLE_STATE)):
                    # have grasped object and delivering......
                    print("state change from", actual_state_message, "to", msg.feedback.message, "object type ", last_grasping_target_type)
                    grasp_positions[grasp_num, 0] = last_grasp_position[0, 0]
                    grasp_positions[grasp_num, 1] = last_grasp_position[0, 1]
                    grasp_positions[grasp_num, 2] = last_grasp_position[0, 2]
                    grasp_positions[grasp_num, 3] = last_grasp_position[0, 3]
                    grasp_positions[grasp_num, 4] = last_grasp_position[0, 4]
                    grasp_num += 1

                actual_state_id = msg.feedback.stage
                actual_state_start = getStampTimeSec(msg.header.stamp)
                actual_state_message = msg.feedback.message

        if topic == mavros_state_topic:
            if msg.armed and armint_z == 0:
                armint_z = last_rtk_pos[0, 3]

            # print(msg.feedback.message)

        readedTopicsNums[topic] += 1

    grasp_positions = grasp_positions[0:grasp_num, :]
    print("readed topics:", readedTopicsNums)
    return grasp_positions


def loadUAVPositionOfGrasps(bag, uav_name):
    print("loadUAVPositionOfGrasps")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    grasping_feedback = "/" + uav_name + "/landing_object_controller/feedback"
    grasping_object_current_target = "/" + uav_name + "/mbzirc_detector/object_array"
    topics_to_read = [odom_topic, grasping_feedback, grasping_object_current_target]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)
    num_landing_feedbacks = bag.get_message_count(grasping_feedback)

    IDLE_STATE = 0
    ALIGN_STATE = 1
    LAND_STATE = 2
    WAIT_STATE = 3
    ALIGN2_STATE = 4
    GRASP_STATE = 5
    REPEAT_STATE = 6
    TAKEOFF_STATE = 7
    ABOART_STATE = 8
    ALIGN_DROP_STATE = 9
    LAND_TO_DROP_STATE = 10
    DROPPING_STATE = 11
    WAITING_AFTER_DROPPING = 12
    WAIT_1_FAIL = 13
    PREEMPTED_STATE = 14

    OBJECT_LONG_MIDDLE = 0
    OBJECT_RED = 1
    OBJECT_GREEN = 2
    OBJECT_BLUE = 3
    OBJECT_MOVING = 5
    OBJECT_STATIC = 999
    OBJECT_LONG_END = 1000

    feedback_changes = []
    last_position = np.zeros(shape=(1, 4))

    last_grasp_position = np.zeros(shape=(1, 5))

    grasp_positions = np.zeros(shape=(bag.get_message_count(grasping_feedback), 5))
    last_grasping_target_type = 0

    actual_state_id = -1
    actual_state_start = 0
    actual_state_message = ""
    grasp_num = 0
    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            last_position[0, 0] = topic_time
            last_position[0, 1] = msg.pose.pose.position.x
            last_position[0, 2] = msg.pose.pose.position.y
            last_position[0, 3] = msg.pose.pose.position.z

        if topic == grasping_object_current_target:
            # if len(msg.objects) > 0:
            #    typessrt = ""
            #    ifor i in range(len(msg.objects)):
            #        typessrt += str(msg.objects[i].type) + " "
            #    print("types:",typessrt)
            if len(msg.objects) == 1:
                last_grasping_target_type = msg.objects[0].type
                # print("type",msg.objects[0].type)

        if topic == grasping_feedback:

            if actual_state_id == -1:
                actual_state_id = msg.feedback.stage
                actual_state_start = getStampTimeSec(msg.header.stamp)
                actual_state_message = msg.feedback.message

            if(actual_state_id != msg.feedback.stage) :
                if(actual_state_id >= 0 and (actual_state_id != ALIGN2_STATE and msg.feedback.stage == ALIGN2_STATE)):
                    print("change to ALIGN2_STATE with last grasping target ", last_grasping_target_type)

                if(actual_state_id >= 0 and (actual_state_id == GRASP_STATE and msg.feedback.stage == TAKEOFF_STATE)):
                    # have grasped object but not sure about succes
                    print("state change from", actual_state_message, "to", msg.feedback.message, "object type ", last_grasping_target_type)
                    last_grasp_position[0, 0] = last_position[0, 0]
                    last_grasp_position[0, 1] = last_position[0, 1]
                    last_grasp_position[0, 2] = last_position[0, 2]
                    last_grasp_position[0, 3] = last_position[0, 3]
                    last_grasp_position[0, 4] = last_grasping_target_type

                if(actual_state_id >= 0 and (actual_state_id == TAKEOFF_STATE and msg.feedback.stage == IDLE_STATE)):
                    # have grasped object and delivering......
                    print("state change from", actual_state_message, "to", msg.feedback.message, "object type ", last_grasping_target_type)
                    grasp_positions[grasp_num, 0] = last_grasp_position[0, 0]
                    grasp_positions[grasp_num, 1] = last_grasp_position[0, 1]
                    grasp_positions[grasp_num, 2] = last_grasp_position[0, 2]
                    grasp_positions[grasp_num, 3] = last_grasp_position[0, 3]
                    grasp_positions[grasp_num, 4] = last_grasp_position[0, 4]
                    grasp_num += 1

                actual_state_id = msg.feedback.stage
                actual_state_start = getStampTimeSec(msg.header.stamp)
                actual_state_message = msg.feedback.message

            # print(msg.feedback.message)

        readedTopicsNums[topic] += 1

    grasp_positions = grasp_positions[0:grasp_num, :]
    print("readed topics:", readedTopicsNums)
    return grasp_positions

def loadSMACHStatesBetweenTime(bag, uav_name, start_time, end_time):
    print("loadSMACHStatesBetweenTime")
    odom_topic = "/" + uav_name + "/mrs_odometry/new_odom"
    grasping_feedback = "/" + uav_name + "/mbzirc_communication/diagnostics"
    topics_to_read = [odom_topic, grasping_feedback]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)
    num_landing_feedbacks = bag.get_message_count(grasping_feedback)
    positions_zero_trust = []



    feedback_changes = []
    last_position = np.zeros(shape=(1, 4))


    actual_state_id = -1
    actual_state_start = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg
        if topic == odom_topic:
            topic_time = getStampTimeSec(msg.header.stamp)
            # print("topic_time",topic_time)
            if topic_time > start_time and topic_time < end_time:
                last_position[0, 0] = topic_time
                last_position[0, 1] = msg.pose.pose.position.x
                last_position[0, 2] = msg.pose.pose.position.y
                last_position[0, 3] = msg.pose.pose.position.z
            if topic_time >= end_time:
                break

        if topic == grasping_feedback:
            if actual_state_id == -1:
                actual_state_id = msg.state_id
                actual_state_start = getStampTimeSec(msg.stamp)

            if(actual_state_id != msg.state_id) :
                feedback_changes.append([actual_state_start, getStampTimeSec(msg.stamp), actual_state_id]);
                print("state change from", actual_state_id, "to", msg.state_id)
                actual_state_id = msg.state_id
                actual_state_start = getStampTimeSec(msg.stamp)





        readedTopicsNums[topic] += 1

    if(actual_state_id > 0) :
        feedback_changes.append([actual_state_start, last_position[0, 0], actual_state_id]);

    print("readed topics:", readedTopicsNums)
    return feedback_changes


def loadObjectDetection(bag, uav_name, start_time, end_time):
    print("loadObjectDetection")
    print("end_time", end_time)
    object_position_topic = "/" + uav_name + "/mbzirc_detector/objects"
    topics_to_read = [object_position_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    LARGE_OBJECT = 0
    RED_OBJECT = 1
    GREEN_OBJECT = 2
    BLUE_OBJECT = 3

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    object_positions = np.zeros(shape=(bag.get_message_count(object_position_topic), 5))

    object_positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg

        # print("topic_time",topic_time)

        if topic == object_position_topic :
            topic_time = getStampTimeSec(msg.timestamp)
            if topic_time > start_time and topic_time < end_time :
                object_positions[object_positions_num, 0] = topic_time
                object_positions[object_positions_num, 1] = msg.absolute.position.x
                object_positions[object_positions_num, 2] = msg.absolute.position.y
                object_positions[object_positions_num, 3] = msg.absolute.position.z
                object_positions[object_positions_num, 4] = msg.type
                object_positions_num += 1

            if topic_time >= end_time:
                break

        readedTopicsNums[topic] += 1

    print("cross_positions_num", object_positions_num)
    cross_positions = object_positions[0:(object_positions_num), :]
    print("readed topics:", readedTopicsNums)
    return cross_positions

def loadProfilerTopics(bag, uav_name, topic_name, start_time, end_time):

    profiler_topic = "/" + uav_name + "/" + topic_name
    topics_to_read = [profiler_topic]
    print(topics_to_read)

    for topic in topics_to_read:
        profiler_msg_number = bag.get_message_count(topic);
    start_stamp = bag.get_start_time()

    profiler_msgs = [ ProfilerUpdate() for _ in range(profiler_msg_number)]

    msg_index = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg

        if topic == profiler_topic :
            topic_time = getStampTimeSec(msg.stamp)
            if topic_time > start_time and topic_time < end_time:
                # print("add",topic_time)
                profiler_msgs[msg_index] = msg
                msg_index += 1

            if topic_time >= end_time:
                break

    profiler_data = profiler_msgs[0:(msg_index)]
    # print("readed topics:", msg_index)
    return profiler_data

def loadObjectPositions(bag, uav_name, start_time, end_time):
    print("loadObjectPositions")
    print("start_time", start_time)
    print("end_time", end_time)
    object_position_topic = "/" + uav_name + "/landing_object_controller/current_target_debug"
    topics_to_read = [object_position_topic]
    print(topics_to_read)
    for topic in topics_to_read:
        print(topic, "has message count", bag.get_message_count(topic))
    start_stamp = bag.get_start_time()
    print("bag start time:", start_stamp)

    readedTopicsNums = dict.fromkeys(topics_to_read, 0)

    object_positions = np.zeros(shape=(bag.get_message_count(object_position_topic), 4))

    object_positions_num = 0

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        # print msg

        # print("topic_time",topic_time)

        if topic == object_position_topic :
            topic_time = getStampTimeSec(msg.header.stamp)
            if topic_time > start_time and topic_time < end_time and (not (msg.pose.position.x == 0 and msg.pose.position.y == 0)):
                # print("add",topic_time)
                object_positions[object_positions_num, 0] = topic_time
                object_positions[object_positions_num, 1] = msg.pose.position.x
                object_positions[object_positions_num, 2] = msg.pose.position.y
                object_positions[object_positions_num, 3] = msg.pose.position.z
                object_positions_num += 1

            if topic_time >= end_time:
                break

        readedTopicsNums[topic] += 1

    print("cross_positions_num", object_positions_num)
    cross_positions = object_positions[0:(object_positions_num), :]
    print("readed topics:", readedTopicsNums)
    return cross_positions


def saveImages(bag, uav_name, image_topic, save_dir, time_image_name=False, between_time=False, start_time=0, end_time=0):
    image_type = ".jpg"
    index_format = "06d"
    image_index = 0
    compressed_topic = False
    if 'compressed' in image_topic:
        compressed_topic = True
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    if not save_dir[-1] != '/':
        save_dir = save_dir + '/'
    # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
    bridge = CvBridge()
    full_topic_name = '/' + uav_name + image_topic
    print('num camera topics', bag.get_message_count(full_topic_name))
    for topic, msg, t in bag.read_messages():
        topic_parts = topic.split('/')

        # first part is empty string
        if topic == full_topic_name :
            topic_time = getStampTimeSec(msg.header.stamp)
            if (not between_time or (between_time and topic_time >= start_time and topic_time <= end_time)):
                try:
                    if compressed_topic:
                        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                    else:
                        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

                    # cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
                    # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except e:
                    print(e)
                # timestr = "%.3f" % msg.header.stamp.to_sec()
                if(image_index % 1000 == 0):
                    print("saved ", image_index, "images")

                image_name = str(save_dir) + "/" + topic_parts[1] + "-" + format(image_index, index_format) + image_type
                if time_image_name:
                    image_name = str(save_dir) + "/" + topic_parts[1] + "-" + str(t) + "s" + image_type
                cv2.imwrite(image_name, cv_image)
                image_index = image_index + 1

def get_info(bag, compressed_topic, topic=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    size = (0, 0)
    times = []
    bridge = CvBridge()
    # read the first message to get the image size
    print("get images from " + topic)
    msg = bag.read_messages(topics=topic).next()[1]
    # size = (msg.width, msg.height)

    # now read the rest of the messages for the rates
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)  # , raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())

        if compressed_topic:
            img = np.asarray(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
            size = (img.shape[1], img.shape[0])
            # print(img.shape)
        else:
            img = np.asarray(bridge.imgmsg_to_cv2(msg, "bgr8"))
            size = (img.shape[1], img.shape[0])
            # print(img.shape)

    diffs = 1 / np.diff(times)
    return np.median(diffs), min(diffs), max(diffs), size, times

def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision * intervals / min(intervals)))

def write_frames(bag, writer, total, compressed_topic, topic=None, nframes=1, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    count = 1
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)
    for (topic, msg, time), reps in izip(iterator, nframes):
        print('\rWriting frame %s of %s at time %s' % (count, total, time))
        if compressed_topic:
            img = np.asarray(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
        else:
            img = np.asarray(bridge.imgmsg_to_cv2(msg, "bgr8"))
        for rep in range(reps):
            writer.write(img)
        count += 1

def savevideo(bag, uav_name, image_topic, outfile):
    print('Calculating video properties')
    compressed_topic = False
    if 'compressed' in image_topic:
        compressed_topic = True
    full_topic_name = '/' + uav_name + image_topic
    rate, minrate, maxrate, size, times = get_info(bag, compressed_topic, full_topic_name)
    precision = 10
    encoding = 'bgr8'
    nframes = calc_n_frames(times, precision)

    # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
    writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'XVID'), np.ceil(maxrate * precision), size)
    print('Writing video to', outfile)

    write_frames(bag, writer, len(times), compressed_topic, topic=full_topic_name, nframes=nframes, encoding=encoding)
    writer.release()
    print('\n')
