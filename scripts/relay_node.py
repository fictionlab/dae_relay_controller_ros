#!/usr/bin/env python3

import rospy
import time

from dae_relay_controller_ros.srv import (
    SetRelay,
    GetAllRelays,
    SetAllRelays,
    SetRelayResponse,
    GetAllRelaysResponse,
    SetAllRelaysResponse,
)
import dae_RelayBoard
from dae_RelayBoard.dae_RelayBoard_Common import Denkovi_Exception


def set_callback(msg):
    try:
        dr.setState(msg.relay_number, msg.state)
    except Denkovi_Exception as e:
        rospy.logerr(f"Failed to set relay state: {e}")
        return SetRelayResponse(False)

    time.sleep(0.5)
    resp = dr.getStates()

    if resp[msg.relay_number] == msg.state:
        is_ok = True
    else:
        is_ok = False

    return SetRelayResponse(is_ok)


def get_callback(msg):
    state = dr.getStates()
    state_arr = [False] * len(state)

    for tmp in state:
        state_arr[tmp - 1] = state[tmp]

    return GetAllRelaysResponse(True, state_arr)


def set_all_callback(msg):
    is_ok = True
    dr.setAllStatesOn() if msg.state else dr.setAllStatesOff()

    resp = dr.getStates()
    print(resp)

    for tmp in resp:
        if resp[tmp] != msg.state:
            is_ok = False

    return SetAllRelaysResponse(is_ok)


try:
    rospy.init_node("relay_node")

    set_relay_srv = rospy.Service("~set_relay", SetRelay, set_callback)
    set_all_relays = rospy.Service("~get_all_relays", GetAllRelays, get_callback)
    get_relay = rospy.Service("~set_all_relays", SetAllRelays, set_all_callback)

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
