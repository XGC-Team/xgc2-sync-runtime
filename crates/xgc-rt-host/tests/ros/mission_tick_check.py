#!/usr/bin/env python3
"""Plays the operator and the controller for one robot, and reads the local
FormationTick the aggregator publishes for the unchanged DMPC planner.
Prints one JSON summary line."""
import json
import sys

import rospy
from formation_generator.msg import FormationTick
from std_msgs.msg import String

ns = sys.argv[1] if len(sys.argv) > 1 else "/uav1"
rospy.init_node("mission_tick_check", disable_signals=True)
ticks = []
rospy.Subscriber(ns + "/formation/mission_tick", FormationTick,
                 lambda m: ticks.append((m.trigger.sequence_id, m.trigger.trigger_time.to_sec(), m.rolling, m.mission_time,
                                         rospy.Time.now().to_sec())))
cmd = rospy.Publisher("/command", String, queue_size=5)
state = rospy.Publisher(ns + "/custom/statustext", String, queue_size=5)
rospy.sleep(1.0)
end = rospy.Time.now().to_sec() + 3.0
sent_start = False
while rospy.Time.now().to_sec() < end:
    state.publish(String("Custom1"))
    if not sent_start and ticks:
        cmd.publish(String("start"))
        sent_start = True
    rospy.sleep(0.05)
print(json.dumps({"ticks": ticks}), flush=True)
