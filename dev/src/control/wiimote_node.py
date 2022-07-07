#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pyright: reportMissingImports=false
# ---------------------------------------------------------------------
# ROS node module to handle robot control using a wiimote
# Supported robot for this module is PMI only...
# 
# (C) Supaero Robotic Club 2022

import time
import rospy
import cwiid

from std_msgs.msg      import Int16
from geometry_msgs.msg import Quaternion

_NODENAME_ = "[WII]"

MIN = 0
MID = 50
MAX = 100

HOME_SIDE = 0
AWAY_SIDE = 1


def LOG(msg):
    rospy.loginfo(f"{_NODENAME_} {msg}")


def rescale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    
    val: float or int
    src: tuple 
    dst: tuple
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def nunchuk(val, cor):
    c_bot, c_mid, c_top, delta = cor
    if abs(val-c_mid) <= delta:
        return MID
    if val < c_mid:
        return rescale(val, [c_bot, c_mid], [MIN, MID])
    if val > c_mid:
        return rescale(val, [c_mid, c_top], [MID, MAX])
    raise RuntimeError("Should not get here...")


class WiiControlNode:

    def __init__(self):
        rospy.init_node('WiiControlNode')
        LOG("Initializing Wii Control Node.")

        # --- Publishers & subscribers
        self.pub_res = rospy.Publisher("/res_request", Int16, queue_size=10, latch=False)
        self.pub_arm = rospy.Publisher("/arm_request", Int16, queue_size=10, latch=False)
        self.pub_nextpos = rospy.Publisher("/nextPositionTeensy", Quaternion, queue_size=10, latch=False)
        self.pub_grab_statue = rospy.Publisher("/grab_statue_request", Int16, queue_size=10, latch=True)
        self.pub_drop_statue = rospy.Publisher("/drop_statue_request", Int16, queue_size=10, latch=True)
        self.pub_drop_replic = rospy.Publisher("/drop_replic_request", Int16, queue_size=10, latch=True)

        # --- Connection to Wiimote
        print("Press 1+2 on Wiimote to connect...")
        self.wiimote = None
        connection_attemps = 1
        while not self.wiimote:
            try:
                self.wiimote = cwiid.Wiimote()
            except RuntimeError:
                print("RuntimError, failure to connect, please try again.")
                connection_attemps += 1
        LOG("Connection to wiimote successful, {} attemps needed.".format(connection_attemps))

        # --- Buttons variables         
        self.side_sel = [False, False]
        self.res = False
        self.arm = False
        self.drop_replic = False
        self.grab_statue = False
        self.drop_statue = False

        self.coeff_speed = 2
        self.t_upd_speed = 0
        
        # --- Config wiimote report button presses & accel
        self.wiimote.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_NUNCHUK
        self.wiimote.led = (1 << (self.coeff_speed+1)) - 1

    def unitstep(self):
        # --- Nunchuk constants (TODO: check values)
        nunchuk_v_consts = (0, 131, 255, 1)
        nunchuk_h_consts = (1, 127, 254, 1)

        max_speed = 255

        # --- Check for any move cmd from nunchuk & publish
        if 'nunchuk' in self.wiimote.state:
            vert = self.wiimote.state['nunchuk']['stick'][1]
            hori = self.wiimote.state['nunchuk']['stick'][0]
            corr_vert = nunchuk(vert, nunchuk_v_consts)
            corr_hori = nunchuk(hori, nunchuk_h_consts)
            speed = (corr_vert - 50)*2 / (5-self.coeff_speed) * max_speed /100
            angle = -(corr_hori - 50)*2 / (5-self.coeff_speed) * max_speed /100 /2
        else:
            speed = angle = 0

        msg_pos = Quaternion()
        msg_pos.x = min(max(speed-angle,-255),255)
        msg_pos.y = min(max(speed+angle,-255),255)
        msg_pos.z = 0
        msg_pos.w = 4  # cmd for remote control ^^
        self.pub_nextpos.publish(msg_pos)

        time.sleep(0.1)

        # --- Check for buttons events | to change as actions change
        wiimote_buttons = self.wiimote.state['buttons']
        nunchuk_buttons = self.wiimote.state['nunchuk']['buttons']
        
        if wiimote_buttons & cwiid.BTN_A != 0:
            if not self.grab_statue:
                self.pub_grab_statue.publish(data=1)
                self.grab_statue = True
        else:
            self.grab_statue = False
        
        if wiimote_buttons & cwiid.BTN_B != 0:
            if not self.drop_statue:
                self.pub_drop_statue.publish(data=1)
                self.drop_statue = True
        else:
            self.drop_statue = False

        if wiimote_buttons & cwiid.BTN_1 != 0:
            if not self.drop_replic:
                self.pub_drop_replic.publish(data=1)
                self.drop_replic = True
        else:
            self.drop_replic = False

        if wiimote_buttons & cwiid.BTN_2 != 0:
            if not self.drop_replic:
                self.pub_drop_replic.publish(data=1)
                self.drop_replic = True
        else:
            self.drop_replic = False

        if wiimote_buttons & cwiid.BTN_LEFT != 0:
            self.side_sel[AWAY_SIDE] = True

        if wiimote_buttons & cwiid.BTN_RIGHT != 0:
            self.side_sel[HOME_SIDE] = True

        if nunchuk_buttons == 1: # Z btn
            if not self.res:
                for k in range(len(self.side_sel)):
                    if not self.side_sel[k]:
                        continue
                    self.pub_res.publish(data=k)
                LOG("Res request.")
                self.res = True
        else:
            self.res = False
   
        if nunchuk_buttons == 2: # C btn
            if not self.arm:
                for k in range(len(self.side_sel)):
                    if not self.side_sel[k]:
                        continue
                    self.pub_arm.publish(data=k)
                LOG("Arm request.")
                self.arm = True
        else:
            self.arm = False

        upd_speed_delay = 0.5

        if wiimote_buttons & cwiid.BTN_MINUS != 0 and time.time() - self.t_upd_speed > upd_speed_delay:
            self.coeff_speed = max(self.coeff_speed - 1, 0)
            self.wiimote.led = (1 << (self.coeff_speed)) - 1
            self.t_upd_speed = time.time()

        if wiimote_buttons & cwiid.BTN_PLUS != 0 and time.time() - self.t_upd_speed > upd_speed_delay:
            self.coeff_speed = min(self.coeff_speed + 1, 4)
            self.wiimote.led = (1 << (self.coeff_speed)) - 1
            self.t_upd_speed = time.time()


    def mainloop(self):
        while True:
            # Refreshing rate = 10Hz 
            time.sleep(0.1)
            # Run one step 
            self.unitstep()


if __name__ == '__main__':
    node = WiiControlNode()
    node.mainloop()