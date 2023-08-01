import rospy, rostopic
from mavros_msgs import *
from mavros_msgs.srv import ParamPull, ParamPullRequest, ParamPullResponse
from mavros_msgs.srv import ParamGet, ParamGetRequest, ParamGetResponse
import sys
import os
import json

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

def print_dual(text):
    print(text)
    print(text, file=open('/tmp/pixhawk_config_tmp.txt', 'a'))


if os.path.exists("/tmp/pixhawk_config_tmp.txt"):
  os.remove("/tmp/pixhawk_config_tmp.txt")

print(BOLD + "This script will test Pixhawk connections and parameters" + END)
print(BOLD + "Mavros and ROS is required for this step, if you did not install the MRS UAV System, this will not work" + END)
print(BOLD + "This script can run for up to a minute, do not interrupt it" + END)

rospy.init_node('pixhawk_tester')

state_hz = rostopic.ROSTopicHz(-1)
rospy.sleep(1)
s = rospy.Subscriber('/uav1/mavros/state', rospy.AnyMsg, state_hz.callback_hz, callback_args='/state')
rospy.sleep(1)
output_hz = state_hz.get_hz('/state')

if output_hz is None:
    hz = 0.0
else:
    hz = output_hz[0]

if hz > 90 and hz < 120:
    print_dual("Pixhawk hearbeat rate: " + str(round(hz, 1)) + GREEN + " PASS" + END)
else:
    print_dual(BOLD + "Pixhawk hearbeat rate: " + str(round(hz, 1)) + RED + " FAIL" + END)
    print_dual(RED + BOLD + "Pixhawk state (heartbeat) message should run at 100Hz!" + END)
    print_dual(RED + BOLD + "Check that Pixhawk SD card config is correct and up to date!" + END)
state_hz.print_hz


param_client_pull = rospy.ServiceProxy('/uav1/mavros/param/pull', ParamPull)
req = ParamPullRequest()
req.force_pull = 1
resp = param_client_pull(req)
if resp.success:
    print_dual("Parameters pulled - pixhawk connection " + GREEN + "PASS" + END)
else:
    print_dual(BOLD + RED +"Parameter pull FAILED! Check connection to Pixhawk, check that both RX and TX are connected!")

param_client_get = rospy.ServiceProxy('/uav1/mavros/param/get', ParamGet)
f = open('px4_params.json')
data = json.load(f)
for i in data:
    req = ParamGetRequest()
    req.param_id = i
    resp = param_client_get(req)
    if resp.success:
        if str(resp.value.integer) in data[i]['valid_values'] or str(resp.value.real) in data[i]['valid_values']:
            print_dual("Parameter " + i + GREEN + " PASS" + END)
        else:
            print_dual("Parameter " + i + RED + " FAIL " + END + BOLD + " " + data[i]['error_message'] + END)
    else:
        print_dual(RED + BOLD + "Failed to check parameter " + i + END + BOLD + " " + data[i]['error_message'] + END)
      

print_dual("-----------------Check Finished-----------------------")
