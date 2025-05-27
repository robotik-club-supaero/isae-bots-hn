#! /usr/bin/python
# -*- coding: utf-8 -*-

import sys
import math
from threading import Thread
import time

from consolemenu import *
from consolemenu.items import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from br_messages.msg import DisplacementOrder, Command, Position, Point

class BrTestNode(Node):
    def __init__(self):
        super().__init__("br_test")
        
        self.pub_reset = self.create_publisher(Position, "/br/resetPosition", 10)
        self.pub_go_to = self.create_publisher(DisplacementOrder, "/br/goTo", 10)
        self.pub_command = self.create_publisher(Command, "/br/command", 10)
        self.pub_stop = self.create_publisher(Empty, "/br/stop", 10)

    def pubResetPosition(self, x=0., y=0., theta=0.):
        msg = Position(x=float(x), y=float(y), theta=float(theta))
        self.pub_reset.publish(msg)

    def pubGoTo(self, x, y, theta=None, backward=False):
        msg = DisplacementOrder()
        msg.path = [Point(x=float(x), y=float(y))]
        if theta is not None:
            msg.theta = float(theta)
            msg.kind = DisplacementOrder.FINAL_ORIENTATION
        if backward:
            msg.kind |= DisplacementOrder.REVERSE
        
        self.pub_go_to.publish(msg)

    def pubRotate(self, theta):
        msg = DisplacementOrder()
        msg.kind = DisplacementOrder.FINAL_ORIENTATION
        msg.theta = float(theta)
        self.pub_go_to.publish(msg)

    def pubStop(self):
        self.pub_stop.publish(Empty())

    def pubCommand(self, linear, angular):
        msg = Command()
        msg.linear = 255.*float(linear)
        msg.angular = 255.*float(angular)
        self.pub_command.publish(msg)

    @staticmethod
    def askPosition():
        x = input("x = ")
        y = input("y = ")
        return x, y 

    def askForward(self):
        d = float(input("Distance (in millimeters, can be negative for backward)? "))
        self.pubResetPosition()
        time.sleep(0.1)
        self.pubGoTo(d, 0, backward=d<0)

    def askRotate(self, theta=None):    
        if theta is None:
            theta = input("theta = ")
        self.pubResetPosition()
        time.sleep(0.1)
        self.pubRotate(theta)

    def askGoTo(self):
        x, y = BrTestNode.askPosition()
        theta = input("theta (leave empty to disable final orientation) = ")
        if theta == "": theta = None
        self.pubResetPosition()
        time.sleep(0.1)
        self.pubGoTo(x, y, theta)

    def askPath(self):
        msg = DisplacementOrder()
        msg.path = [
            Point(x=400., y=200.),
            Point(x=700., y=1920.)
        ]
        msg.theta = 0.
        msg.kind = DisplacementOrder.ALLOW_CURVE | DisplacementOrder.FINAL_ORIENTATION

        self.pubResetPosition(10, 100)
        time.sleep(0.1)
        self.pub_go_to.publish(msg)

def main():
    rclpy.init(args=sys.argv)
    node = BrTestNode()

    try:
        th = Thread(target = lambda: rclpy.spin(node))
        th.start()


        #Main menu
        menu = ConsoleMenu("Main menu")

        #Moving bot
        pos_ctrl_sub = ConsoleMenu("Position control - position will be reset")

        pos_ctrl_items = []
        pos_ctrl_items.append(FunctionItem("Go straight", node.askForward))
        pos_ctrl_items.append(FunctionItem("Go to...", node.askGoTo))
        pos_ctrl_items.append(FunctionItem("Follow preset path", node.askPath))
        pos_ctrl_items.append(FunctionItem("Do a 180 turn" , node.askRotate, [math.pi]))
        pos_ctrl_items.append(FunctionItem("Do an orientation of...", node.askRotate))

        for item in pos_ctrl_items :
            pos_ctrl_sub.append_item(item)

        #Direct_command
        speed_ctrl_sub = ConsoleMenu("Speed control")

        speed_ctrl_items = []
        speed_ctrl_items.append(FunctionItem("Go forward (at 80%)", node.pubCommand, [0.8, 0.]))
        speed_ctrl_items.append(FunctionItem("Go backward (at 80%)", node.pubCommand, [-0.8, 0.]))
        speed_ctrl_items.append(FunctionItem("Rotation clockwise (at 80%)", node.pubCommand, [0., -0.8]))
        speed_ctrl_items.append(FunctionItem("Rotation counter-clockwise (at 80%)", node.pubCommand, [0., 0.8]))
        speed_ctrl_items.append(FunctionItem("Stop the bot", node.pubCommand, [0., 0.]))
        speed_ctrl_items.append(FunctionItem("Stop the bot (emergency)", node.pubStop))

        for item in speed_ctrl_items :
            speed_ctrl_sub.append_item(item)

        pos_ctrl_item = SubmenuItem("Position control", pos_ctrl_sub)
        speed_ctrl_item = SubmenuItem("Speed control", speed_ctrl_sub)

        menu.append_item(pos_ctrl_item)
        menu.append_item(speed_ctrl_item)

        menu.show()

    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()