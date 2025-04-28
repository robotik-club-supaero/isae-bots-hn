import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

from config import COLOR
from config.qos import default_profile

from .lcd_lib import lcd

class LCDNode(Node):

    def __init__(self):
        super().__init__("ISB")

        self.match_started = False
        self.color = 0
        self.strat = 0
        self.init_pos = 0
        self.score = 0

        self.lcd = lcd()
        self.lcd.lcd_clear()

        self.subScore = self.create_subscription(Int16, "/game/color", self.cb_color, default_profile)
        self.subScore = self.create_subscription(Int16, "/game/strat", self.cb_strat, default_profile)
        self.subStart = self.create_subscription(Int16, "/game/start", self.cb_start, default_profile)
        self.subInitPos = self.create_subscription(Int16, '/game/init_pos', self.cb_init_pos, default_profile)

        self.subScore = self.create_subscription(Int16, "/game/score", self.cb_score, default_profile)

        self.update_display()
        self.get_logger().info("LCD node initialized")
        
    def cb_color(self, msg):
        self.color = msg.data
        self.update_display()

    def cb_strat(self, msg):
        self.strat = msg.data
        self.update_display()

    def cb_start(self, msg):
        if msg.data == 1:
            self.match_started = True
            self.update_display()

    def cb_init_pos(self, msg):
        self.init_pos = msg.data
        self.update_display()

    def cb_score(self, msg):
        self.score = msg.data
        self.update_display()

    def update_display(self):
        self.lcd.lcd_clear()
        if self.match_started:
            self.lcd.lcd_display_string("SCORE: " + str(self.score), line=1)
        else:
            self.lcd.lcd_display_string("STRAT: " + str(self.strat), line=1)
            self.lcd.lcd_display_string(COLOR[self.color % 2] + f" (ZONE {self.init_pos})", line=2)

    def run(self):     
        try:
            rclpy.spin(self)
        finally:
            self.lcd.lcd_clear()

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = LCDNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()