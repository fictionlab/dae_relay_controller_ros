#!/usr/bin/env python

from tokenize import String
import rospy
import time

import argparse

from dae_relay_controller_ros.srv import *
import dae_RelayBoard


def set_callback(msg):
    dr.setState(msg.relay_number, msg.state)
    time.sleep(0.5)
    resp = dr.getStates()

    if resp[msg.relay_number] == msg.state:
        is_ok = True
    else:
        is_ok = False

    return SetRelayResponse(is_ok, "")


def get_callback(msg):
    state = dr.getStates()
    state_arr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    for tmp in state:
        state_arr[tmp - 1] = state[tmp]

    return GetRelayResponse(state_arr, "")


def set_all_callback(msg):
    is_ok = True
    dr.setAllStatesOn() if msg.state else dr.setAllStatesOff()

    resp = dr.getStates()
    print(resp)

    for tmp in resp:
        if resp[tmp] != msg.state:
            is_ok = False

    return SetAllRelaysResponse(is_ok, "")


try:
    rospy.init_node("relay_node")

    set_relay_srv = rospy.Service("relay/set_relay", SetRelay, set_callback)
    set_all_relays = rospy.Service("relay/get_relay", GetRelay, get_callback)
    get_relay = rospy.Service("relay/set_relay/all", SetAllRelays, set_all_callback)

    rospy.loginfo("Relay node started!")

except rospy.ROSInterruptException as e:
    rospy.loginfo("Relay node error!")
    rospy.logerr(e)

device = rospy.get_param("~device", "/dev/relay")
type = rospy.get_param("~module_type", "type16")

dr = dae_RelayBoard.DAE_RelayBoard(type)
dr.initialise(device)

rospy.loginfo("Device type: %s", type)
rospy.loginfo("Device path: %s", device)

dr.setAllStatesOff()

rospy.spin()
