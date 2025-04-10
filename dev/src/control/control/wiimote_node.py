#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 

# pyright: reportMissingImports=false
#
# ROS node module to handle robot control using a wiimote
# Supported robot for this module is PMI only...

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import sys
import time
from time import perf_counter
import rclpy
from rclpy.node import Node
import cwiid
from enum import IntEnum, Enum
from std_msgs.msg      import  Bool
from br_messages.msg import Command
from config.qos import default_profile

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################


class STICK_SCALE(IntEnum):
    MIN = 0
    MID = 50
    MAX = 100

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

def rescale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    
    Args:
        val: float or int
        src: tuple 
        dst: tuple
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def nunchuk(val, cor):
    c_bot, c_mid, c_top, delta = cor
    if abs(val-c_mid) <= delta:
        # TODO : there is something to do here
        return STICK_SCALE.MID
    if val < c_mid:
        return rescale(val, [c_bot, c_mid], [STICK_SCALE.MIN, STICK_SCALE.MID])
    if val > c_mid:
        return rescale(val, [c_mid, c_top], [STICK_SCALE.MID, STICK_SCALE.MAX])
    raise RuntimeError("Should not get here...")


class WiiControlNode(Node):
    """
    ROS node WII CONTROL for remote control of the robot by wiimote.
    """

    def __init__(self):
        super().__init__("wii_node")
        self.log_info("Initializing WII node ...")
        # -- Publishers & subscribers
        
        self.command_pub = self.create_publisher(Command, "/br/command", default_profile)
        self.idle_pub = self.create_publisher(Bool, "/br/idle", default_profile)

        # -- Connection to Wiimote
        self.wiimote = None
        self.wiimote_connect()

        # -- Buttons variables
               
        # -- Dynamic variables
        self.coeff_speed = 2
        self.t_upd_speed = 0
        self.t_btn_a = 0
        self.t_btn_b = 0

        # --- Config wiimote report button presses & accel
        self.wiimote.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_NUNCHUK
        self.wiimote.led = (1 << (self.coeff_speed+1)) - 1

        self.msg_pos = Command()
        self.msg_idle = Bool()

    def wiimote_connect(self):
        """
        Method to connect to the wiimote
        """
        self.log_warn("Press 1+2 on Wiimote to connect ...")
        connection_attempts = 1
        while not self.wiimote:
            try:
                self.wiimote = cwiid.Wiimote()
            except RuntimeError:
                self.log_errs("Failure to connect, please try again.")
                connection_attempts += 1
        self.log_info(f"Success, connection established, {connection_attempts} attempts needed.")

    def unitstep(self):
        # --- Nunchuk constants (TODO: check values)
        nunchuk_v_consts = (33, 126, 218, 1)
        nunchuk_h_consts = (24, 120, 220, 1)
        max_speed = 255

        # --- Check for any move cmd from nunchuk & publish
        if 'nunchuk' in self.wiimote.state:
            vert = self.wiimote.state['nunchuk']['stick'][1]
            hori = self.wiimote.state['nunchuk']['stick'][0]
            corr_vert = nunchuk(vert, nunchuk_v_consts)
            corr_hori = nunchuk(hori, nunchuk_h_consts)
            speed = (corr_vert - 50)*2 / (5-self.coeff_speed) * max_speed / 100
            angle = (corr_hori - 50)*2 / (5-self.coeff_speed) * max_speed / 100 / 2
        else:
            speed = angle = 0
        self.msg_pos.linear = float(min(max(speed, -max_speed), max_speed))
        self.msg_pos.angular = -float(min(max(angle, -max_speed), max_speed))

        # seuillage
        if abs(self.msg_pos.linear) < 10:
            self.msg_pos.linear = 0.
        if abs(self.msg_pos.angular) < 10:
            self.msg_pos.angular = 0.

        self.command_pub.publish(self.msg_pos)

        time.sleep(0.01)

        # --- Check for buttons events | to change as actions change
        wiimote_buttons = self.wiimote.state['buttons']
        nunchuk_buttons = self.wiimote.state['nunchuk']['buttons'] if 'nunchuk' in self.wiimote.state else 0


        if wiimote_buttons & cwiid.BTN_A != 0:
            if self.t_btn_a == 0 or time.time() - self.t_btn_a > 1:
                self.msg_idle.data = True
                self.idle_pub.publish(self.msg_idle)
                self.t_btn_a = time.time()
        else:
            self.t_btn_a = 0

        if wiimote_buttons & cwiid.BTN_B != 0:
            if self.t_btn_b == 0 or time.time() - self.t_btn_b > 1:
                self.msg_idle.data = False
                self.idle_pub.publish(self.msg_idle)
                self.t_btn_b = time.time()
        else:
            self.t_btn_b = 0

        if wiimote_buttons & cwiid.BTN_1 != 0:
            pass
        else:
            pass

        if wiimote_buttons & cwiid.BTN_2 != 0:
            pass
        else:
            pass

        if wiimote_buttons & cwiid.BTN_LEFT != 0:
            pass

        if wiimote_buttons & cwiid.BTN_RIGHT != 0:
            pass

        if nunchuk_buttons & cwiid.NUNCHUK_BTN_Z != 0:
            pass
        else:
            pass

        if nunchuk_buttons & cwiid.NUNCHUK_BTN_C != 0:
            pass
        else:
            pass

        upd_speed_delay = 0.5

        if wiimote_buttons & cwiid.BTN_MINUS != 0 and time.time() - self.t_upd_speed > upd_speed_delay:
            self.coeff_speed = max(self.coeff_speed - 1, 0)
            self.wiimote.led = (1 << (self.coeff_speed)) - 1
            self.t_upd_speed = time.time()

        if wiimote_buttons & cwiid.BTN_PLUS != 0 and time.time() - self.t_upd_speed > upd_speed_delay:
            self.coeff_speed = min(self.coeff_speed + 1, 4)
            self.wiimote.led = (1 << (self.coeff_speed)) - 1
            self.t_upd_speed = time.time()

        
    def log_info(self, msg):
        """
        Standard logs print.
        """
        self.get_logger().info(msg)


    def log_warn(self, msg):
        """
        Warning logs print.
        """
        self.get_logger().warning(msg)


    def log_errs(self, msg):
        """
        Errors logs print.
        """
        self.get_logger().error(msg)


    def mainloop(self):
        """
        ROS node mainloop
        """
        begin = perf_counter()
        while rclpy.ok():
            # Refreshing rate of 20Hz with 1000Hz resolution
            while perf_counter() - begin < 0.1:
                time.sleep(0.001)
            begin = perf_counter()

            self.unitstep()
            rclpy.spin_once(self, timeout_sec=0)

#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = WiiControlNode()
    try:
        node.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()